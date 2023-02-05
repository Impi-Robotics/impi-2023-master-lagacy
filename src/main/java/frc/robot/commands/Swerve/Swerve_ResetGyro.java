// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Swerve_ResetGyro extends InstantCommand {

  SwerveSubsystem swerveSubsystem;
  double angle;

  public Swerve_ResetGyro(SwerveSubsystem swerveSubsystem, double angle) {
    this.swerveSubsystem = swerveSubsystem;
    this.angle = angle;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.setGyroAngle(angle);
  }
}
