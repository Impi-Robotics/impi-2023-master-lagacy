package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ChassisDriveStraight extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final double xSpeed;
    private final double ySpeed;

    public ChassisDriveStraight(SwerveSubsystem swerveSubsystem, double xSpeed, double ySpeed) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerveSubsystem.driveStraight(xSpeed, ySpeed);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
