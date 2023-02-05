package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ChassisSubsystem;

public class ChassisRotateFieldBackward extends InstantCommand {

  private ChassisSubsystem chassisSubsystem;

  public ChassisRotateFieldBackward(ChassisSubsystem chassisSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    addRequirements(chassisSubsystem);
  }

  @Override
  public void initialize() {
    /*
     * Rotates the robot 180 degrees from field relative front
     */
    chassisSubsystem.rotateFieldBackward();
  }
}
