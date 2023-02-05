package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ChassisSubsystem;

public class ChassisTurnToTargetOdometryToggle extends InstantCommand {
  
  private final ChassisSubsystem chassisSubsystem;

  public ChassisTurnToTargetOdometryToggle(ChassisSubsystem chassisSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    addRequirements(chassisSubsystem);
  }

  @Override
  public void initialize() {
    /*
     * Toggle to turn off and on odometry
     */
    chassisSubsystem.turnToTargetOdometry();
  }
}
