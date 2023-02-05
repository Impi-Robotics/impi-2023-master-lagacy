// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ImpiLib2023;
import frc.robot.subsystems.SwerveSubsystem;

public class Swerve_DriveField extends CommandBase {

  SwerveSubsystem swerveSubsystem;
  DoubleSupplier xSpeed;
  DoubleSupplier ySpeed;
  DoubleSupplier rotR;
  DoubleSupplier rotL;
  boolean fieldOriented;

  /** Creates a new Swerve_DriveField. */
  public Swerve_DriveField(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotR, DoubleSupplier rotL, boolean fieldOriented) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotR = rotR;
    this.rotL = rotL;
    this.fieldOriented = fieldOriented;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xValue = ImpiLib2023.parseJoystick(xSpeed) * Constants.CHASSIS.MAX_METERS_PER_SECOND;
    double yValue = ImpiLib2023.parseJoystick(ySpeed) * Constants.CHASSIS.MAX_METERS_PER_SECOND;
    double rotValue = (rotL.getAsDouble() - rotR.getAsDouble()) * 7.5;
    swerveSubsystem.swerveDriveField(xValue, yValue, rotValue, fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
