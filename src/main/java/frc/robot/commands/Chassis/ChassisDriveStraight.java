package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChassisSubsystem;

public class ChassisDriveStraight extends CommandBase {

    private final ChassisSubsystem chassisSubsystem;
    private final double xSpeed;
    private final double ySpeed;

    public ChassisDriveStraight(ChassisSubsystem chassisSubsystem, double xSpeed, double ySpeed) {
        this.chassisSubsystem = chassisSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        addRequirements(chassisSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        chassisSubsystem.driveStraight(xSpeed, ySpeed);
    }

    @Override
    public void end(boolean interrupted) {
        chassisSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
