package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ChassisSubsystem;

public class ChassisRotateFieldRight extends InstantCommand {

  private final ChassisSubsystem chassisSubsystem;
  
  public ChassisRotateFieldRight(ChassisSubsystem chassisSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    addRequirements(chassisSubsystem);
  }

  @Override
  public void initialize() {
    /*
     * Rotates the robot 90 degrees to the right from field relative front
     */
    chassisSubsystem.rotateFieldRight();
  }
}
