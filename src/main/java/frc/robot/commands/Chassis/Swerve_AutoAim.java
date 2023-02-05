// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import org.apache.commons.lang3.function.ToBooleanBiFunction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class Swerve_AutoAim extends CommandBase {
  /** Creates a new Swerve_AutoAim. */
  private SwerveSubsystem swerveSubsystem;
  private LimelightSubsystem limelightSubsystem;

  public Swerve_AutoAim(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.setDesiredAngle(limelightSubsystem.targetXOffset());
    swerveSubsystem.dumbDrive(0., 0., 0.);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {/*boolean start toString(on)ToBooleanBiFunction iFunction on swerve;*/
    swerveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (swerveSubsystem.isTargetCentered(limelightSubsystem.targetXOffset())) {
        return true;
    }
    return false;

  }
}