package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DrivePath extends Command {
    
    private Swerve m_swerve;

    private final double TOLERANCE = 0.2;
    private final double ANGLE_TOLERANCE = 0.75;

    // Trajectory path;
    PathPlannerTrajectory path;

    private PIDController anglePIDController = new PIDController(0.04, 0, 0.001);
    private PIDController xController = new PIDController(1.0, 0, 0);
    private PIDController yController = new PIDController(1.0, 0, 0);

    Timer t;

    public DrivePath(Swerve swerve, String pathName) {
        this.m_swerve = swerve;

        anglePIDController.setTolerance(ANGLE_TOLERANCE);
        anglePIDController.enableContinuousInput(0, 360);
        anglePIDController.setSetpoint(180);
        
        xController.setTolerance(TOLERANCE);
        yController.setTolerance(TOLERANCE);

        

        path = PathPlanner.loadPath(pathName, new PathConstraints(3.5, 2));

        // TrajectoryConfig config = new TrajectoryConfig(1, 0.5);
        // List<Pose2d> points = new ArrayList<Pose2d>();
        // points.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        // points.add(new Pose2d(2.5, 0, Rotation2d.fromDegrees(0)));
        // points.add(new Pose2d(5.95, 2, Rotation2d.fromDegrees(0)));
        // points.add(new Pose2d(5.95, 0, Rotation2d.fromDegrees(0)));
        // // points.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        // path = TrajectoryGenerator.generateTrajectory(points, config);
        
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        t = new Timer();
        t.reset();
        t.start();
        m_swerve.zeroGyro(180);
        Pose2d initialPose = path.getInitialPose();
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            m_swerve.resetOdometry(new Pose2d(-initialPose.getX(), initialPose.getY(), m_swerve.getYaw()));
        } else {
            m_swerve.resetOdometry(new Pose2d(-initialPose.getX(), -initialPose.getY(), m_swerve.getYaw()));
        }
    }

    @Override
    public void execute() {

        double x = m_swerve.getPose().getX();
        double y = m_swerve.getPose().getY();

        PathPlannerState targetState = (PathPlannerState) path.sample(t.get());

        double targetX = -targetState.poseMeters.getX();
        double targetY = -targetState.poseMeters.getY();
        double targetYaw = targetState.holonomicRotation.getDegrees();

        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            targetY = -targetY;
        }

        if (targetYaw < 0) {
            targetYaw += 360;
        }
        
        SmartDashboard.putNumber("path following x", targetX);
        SmartDashboard.putNumber("path following y", targetY);
        SmartDashboard.putNumber("path following angle", targetYaw);

        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);
        // 180 - (targetYaw - 180)
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            anglePIDController.setSetpoint(180 - (targetYaw - 180));
        } else {
            anglePIDController.setSetpoint(targetYaw);
        }

        double yaw = m_swerve.getYaw().getDegrees() % 360;
        if (yaw < 0) {
            yaw += 360;
        }

        double rotation = anglePIDController.calculate(yaw);
        double xCorrection = xController.calculate(x);
        double yCorrection = yController.calculate(y);
        // SmartDashboard.putNumber("absolute yaw", yaw);
        
        m_swerve.driveAuto(
            new Translation2d(xCorrection, yCorrection).times(Constants.Swerve.maxAutoSpeed), 
            -rotation * Constants.Swerve.maxAngularVelocity, 
            true, 
            false
        );
    }

    @Override
    public boolean isFinished() {
        // Pose2d currentTargetPose = ((PathPlannerState) path.sample(t.get())).poseMeters;
        // Pose2d currentPose = m_swerve.getPose();
        // if (currentTargetPose.getX() == path.getEndState().poseMeters.getX() && currentTargetPose.getY() == path.getEndState().poseMeters.getY()) {
        //     SmartDashboard.putBoolean("checking end condition", true);
        //     return Math.hypot(currentPose.getX() - currentTargetPose.getX(), currentPose.getY() - currentTargetPose.getY()) < 0.5;
        // }
        // return false;
        return t.get() > path.getTotalTimeSeconds() + 1.5;

    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0, 
            true, 
            false
        );
    }
}
