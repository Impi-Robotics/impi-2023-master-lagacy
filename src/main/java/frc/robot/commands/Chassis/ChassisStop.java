package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ChassisSubsystem;

public class ChassisStop extends InstantCommand {
  private ChassisSubsystem chassisSubsystem;
  public ChassisStop(ChassisSubsystem chassisSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    addRequirements(chassisSubsystem);
  }

  @Override
  public void initialize() {
    chassisSubsystem.stop();
  }
}
