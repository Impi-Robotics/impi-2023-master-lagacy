package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ImpiLib2023;
import frc.robot.subsystems.SwerveSubsystem;

public class Swerve_DriveStraight extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier ySpeed;
  public Swerve_DriveStraight(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerveSubsystem.driveStraight(-ImpiLib2023.parseJoystick(xSpeed, 0.1), ImpiLib2023.parseJoystick(ySpeed, 0.1));
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
