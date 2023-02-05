package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class ChassisResetGyro extends InstantCommand {

  private SwerveSubsystem swerveSubsystem;
  private double angle;

  public ChassisResetGyro(SwerveSubsystem swerveSubsystem, double angle) {
    this.swerveSubsystem = swerveSubsystem;
    this.angle = angle;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    /*
     * Resets gyro and odometry. This allows us to be more accurate with odometry
     * and updates field relative front.
     */
    swerveSubsystem.setGyroAngle(angle);
  }
}
