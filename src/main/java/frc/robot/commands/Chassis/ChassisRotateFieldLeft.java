package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ChassisSubsystem;

public class ChassisRotateFieldLeft extends InstantCommand {

  private final ChassisSubsystem chassisSubsystem;

  public ChassisRotateFieldLeft(ChassisSubsystem chassisSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    addRequirements(chassisSubsystem);
  }

  @Override
  public void initialize() {
    /*
     * Rotates the robot 90 degrees to the left from field relative front
     */
    chassisSubsystem.rotateFieldLeft();
  }
}
