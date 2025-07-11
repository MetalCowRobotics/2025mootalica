package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlignToSubstation extends Command {
    
    private Swerve m_swerve;

    private PIDController anglePIDController = new PIDController(0.004, 0, 0.000);
    private PIDController xController = new PIDController(.8, 0, 0);
    private PIDController yController = new PIDController(.8, 0, 0);

    Trajectory path;
    TrajectoryConfig substationTrajectoryConfig = new TrajectoryConfig(Constants.Swerve.maxSpeed / 2.0, Constants.Swerve.maxSpeed / 4.0);

    double substationX;
    double substationY;
    
    double targetX;
    double targetY;
    double targetYaw = 0;

    double x;
    double y;
    double yaw;

    private double tolerance = 0.03;

    Timer timer;

    public AlignToSubstation(Swerve swerve, double targetX, double targetY) {
        this.m_swerve = swerve;
        this.substationX = targetX;
        this.substationY = targetY;

        timer = new Timer();
        timer.stop();

        xController.setTolerance(tolerance, 0.1);
        yController.setTolerance(tolerance, 0.1);
        anglePIDController.setTolerance(1, 1);
        anglePIDController.enableContinuousInput(-180, 180);
        

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        double[] initialX = {m_swerve.getPose().getX(), 0};
        double[] initialY = {m_swerve.getPose().getY(), 0};
        Spline.ControlVector initialControlVector = new Spline.ControlVector(initialX, initialY);

        double[] finalX = {substationX, 0.5};
        double[] finalY = {substationY, 0};

        List<Translation2d> interiorPoints = new ArrayList<Translation2d>();

        Spline.ControlVector finalControlVector = new Spline.ControlVector(finalX, finalY);
        path = TrajectoryGenerator.generateTrajectory(initialControlVector, interiorPoints, finalControlVector, substationTrajectoryConfig);
        
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        x = m_swerve.getPose().getX();
        y = m_swerve.getPose().getY();
        targetX = path.sample(timer.get()).poseMeters.getX();
        targetX = path.sample(timer.get()).poseMeters.getY();

        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);
        anglePIDController.setSetpoint(targetYaw);

        yaw = m_swerve.getYaw().getDegrees();

        double rotation = anglePIDController.calculate(yaw);
        double xCorrection = xController.calculate(x);
        double yCorrection = yController.calculate(y);

        if (anglePIDController.atSetpoint()) {
            rotation = 0;
        }
        if (xController.atSetpoint()) {
            xCorrection = 0;
        }
        if (yController.atSetpoint()) {
            yCorrection = 0;
        }
        
        m_swerve.drive(
            new Translation2d(xCorrection, yCorrection).times(Constants.Swerve.maxSpeed), 
            -rotation * Constants.Swerve.maxAngularVelocity, 
            true, 
            false
        );
    }

    @Override
    public boolean isFinished() {
        return timer.get() > path.getTotalTimeSeconds() + 2;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        m_swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            true, 
            false
        );
    }
}
