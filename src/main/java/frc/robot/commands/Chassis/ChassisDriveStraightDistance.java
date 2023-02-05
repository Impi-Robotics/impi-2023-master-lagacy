package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChassisSubsystem;

public class ChassisDriveStraightDistance extends CommandBase {

    private final ChassisSubsystem chassisSubsystem;
    private final double xSpeed;
    private final double ySpeed;
    private final double distance;

    public ChassisDriveStraightDistance(ChassisSubsystem chassisSubsystem, double xSpeed, double ySpeed, double distance) {
        this.chassisSubsystem = chassisSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;

        this.distance = distance;
        addRequirements(chassisSubsystem);
    }

    @Override
    public void initialize() {
        chassisSubsystem.resetDriveEncoderPositions();
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
    if (Math.abs(chassisSubsystem.getDriveStraightEncoderPosition()) > distance) {
      return true;
    }
    return false;
  }
}
