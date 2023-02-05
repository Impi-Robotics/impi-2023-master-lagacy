package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class ChassisStop extends InstantCommand {
  private SwerveSubsystem swerveSubsystem;
  public ChassisStop(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.stop();
  }
}
