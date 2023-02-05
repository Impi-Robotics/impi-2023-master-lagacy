// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Chassis_DriveToTarget extends InstantCommand {
  private SwerveSubsystem swerveSubsystem;
  public Chassis_DriveToTarget(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.driveStraight(1, 0);
  }
}
