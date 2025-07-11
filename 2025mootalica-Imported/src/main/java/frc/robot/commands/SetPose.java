package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class SetPose extends Command {
    
    private Swerve m_swerve;
    private double x;
    private double y;

    public SetPose(Swerve swerve, double x, double y) {
        this.m_swerve = swerve;
        this.x = x;
        this.y = y;
        addRequirements(m_swerve);
    }

    @Override
    public void execute() {
        m_swerve.resetOdometry(new Pose2d(x, y, m_swerve.getYaw()));
        m_swerve.zeroGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
