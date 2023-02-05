package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ChassisSubsystem;

public class ChassisResetGyro extends InstantCommand {

  private ChassisSubsystem chassisSubsystem;
  private double angle;

  public ChassisResetGyro(ChassisSubsystem chassisSubsystem, double angle) {
    this.chassisSubsystem = chassisSubsystem;
    this.angle = angle;
    addRequirements(chassisSubsystem);
  }

  @Override
  public void initialize() {
    /*
     * Resets gyro and odometry. This allows us to be more accurate with odometry
     * and updates field relative front.
     */
    chassisSubsystem.setGyroAngle(angle);
  }
}
