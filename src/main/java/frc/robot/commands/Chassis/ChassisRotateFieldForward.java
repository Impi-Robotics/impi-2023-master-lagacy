package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ChassisSubsystem;

public class ChassisRotateFieldForward extends InstantCommand {

  private final ChassisSubsystem chassisSubsystem;
  
  public ChassisRotateFieldForward(ChassisSubsystem chassisSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    addRequirements(chassisSubsystem);
  }

  @Override
  public void initialize() {
    /*
     * Rotates the robot to front from field relative front
     */
    chassisSubsystem.rotateFieldForward();
  }
}
