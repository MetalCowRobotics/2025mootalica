package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private boolean visionEnabled = true;

    private SwerveDrivePoseEstimator estimator;
    
    private PhotonCamera camera;
    private double accelerationTime = 0.6;
    private final double ODOMETRY_RESET_DISTANCE_THRESHOLD = 2.5;
    private final double ODOMETRY_RESET_TIME_THRESHOLD = 15;
    private double lastObservedTime = -2 * ODOMETRY_RESET_TIME_THRESHOLD;

    private double linearAcceleration = Constants.Swerve.maxSpeed * 1.39 / accelerationTime;
    private double angularAcceleration = Constants.Swerve.maxAngularVelocity / accelerationTime;

    private SlewRateLimiter m_xSlewRateLimiter = new SlewRateLimiter(linearAcceleration, -linearAcceleration, 0);
    private SlewRateLimiter m_ySlewRateLimiter = new SlewRateLimiter(linearAcceleration, -linearAcceleration, 0);
    private SlewRateLimiter m_angleSlewRateLimiter = new SlewRateLimiter(angularAcceleration, -angularAcceleration, 0);

    private int lastTrackedTarget = -1;

    private double speedMultiplier = 1;

    private double lastHeading = 0;
    private PIDController angleHoldingPIDController = new PIDController(0.0004, 0, 0);

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        estimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d(0, 0, getYaw()));

        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

        angleHoldingPIDController.setTolerance(2);
        angleHoldingPIDController.enableContinuousInput(-180, 180);
        
    }

    public void disableVision() {
        visionEnabled = false;
    }

    public void enableVision() {
        visionEnabled = true;
    }

    public void setCrawl() {
        speedMultiplier = 0.5;
    }

    public void setSprint() {
        speedMultiplier = 1.39;
    }

    public void setBase() {
        speedMultiplier = 1;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SmartDashboard.putNumber("multiplier", speedMultiplier);
        double xSpeed = m_xSlewRateLimiter.calculate(translation.getX() * speedMultiplier);
        double ySpeed = m_ySlewRateLimiter.calculate(translation.getY() * speedMultiplier);
        double angularVelocity;
        if (rotation == 0) {
            angularVelocity = angleHoldingPIDController.calculate(getYaw().getDegrees());
            angularVelocity = m_angleSlewRateLimiter.calculate(rotation);
        } else {
            angularVelocity = m_angleSlewRateLimiter.calculate(rotation);
            angleHoldingPIDController.setSetpoint(getYaw().getDegrees());
        }
        
        
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    angularVelocity, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    angularVelocity
                                )
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed * speedMultiplier);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    
    public void driveAuto(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double xSpeed = translation.getX();
        double ySpeed = translation.getY();
        double angularVelocity = rotation;
        angleHoldingPIDController.setSetpoint(getYaw().getDegrees());
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    angularVelocity, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    angularVelocity
                                )
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed * speedMultiplier);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed * speedMultiplier);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        estimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void zeroGyro(double angle){
        gyro.setYaw(angle);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble()) : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public Rotation2d getRoll() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getRoll().getValueAsDouble()) : Rotation2d.fromDegrees(gyro.getRoll().getValueAsDouble());
    }

    public Rotation2d getBalanceAngle() {
        return Rotation2d.fromDegrees(360 - gyro.getRoll().getValueAsDouble());
    }

    private void addVisionMeasurement() {
        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        
        if (null != target) {
            double time = result.getTimestampSeconds();
            Transform3d targetPose = target.getBestCameraToTarget();
            double photonX = targetPose.getX();
            double photonY = targetPose.getY();

            double yaw = getYaw().getDegrees() % 360;
            if (yaw < 0) {
                yaw += 360;
            }

            double photonXAngle = yaw - 180;
            double photonYAngle = yaw + 90 - 180;

            Translation2d photonXVector = new Translation2d(
                photonX * Math.cos(Math.toRadians(photonXAngle)),
                photonX * Math.sin(Math.toRadians(photonXAngle))
            );

            Translation2d photonYVector = new Translation2d(
                photonY * Math.cos(Math.toRadians(photonYAngle)),
                photonY * Math.sin(Math.toRadians(photonYAngle))
            );

            Translation2d apriltagPosition = photonXVector.plus(photonYVector);

            Translation2d robotPosition = apriltagPosition.times(-1.0);

            Pose2d robotPose = new Pose2d(
                robotPosition,
                getYaw()
            );

            if (lastTrackedTarget != target.getFiducialId()) {
                resetOdometry(robotPose);
            }
            estimator.addVisionMeasurement(robotPose, time);
            
            lastTrackedTarget = target.getFiducialId();

            SmartDashboard.putNumber("vision x", robotPose.getX());
            SmartDashboard.putNumber("vision y", robotPose.getY());

        } else {
            lastTrackedTarget = -1;
        }

        SmartDashboard.putBoolean("has targets", result.hasTargets());
        SmartDashboard.putNumber("last tracked target", lastTrackedTarget);
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
        estimator.update(getYaw(), getModulePositions());
        if (visionEnabled) {
            addVisionMeasurement();
        }
        SmartDashboard.putBoolean("vision enabled", visionEnabled);

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("yaw", getYaw().getDegrees());
        Pose2d pose = swerveOdometry.getPoseMeters();
        SmartDashboard.putNumber("x from odometry", pose.getX());
        SmartDashboard.putNumber("y from odometry", pose.getY());
        SmartDashboard.putNumber("charge station angle", getBalanceAngle().getDegrees());
    }
}