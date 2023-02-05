// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Chassis_StrafeToTarget extends InstantCommand {
  private ChassisSubsystem chassisSubsystem;
  private LimelightSubsystem limelightSubsystem;
  public Chassis_StrafeToTarget(ChassisSubsystem chassisSubsystem, LimelightSubsystem limelightSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(chassisSubsystem);
    addRequirements(limelightSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassisSubsystem.strafeToTarget(limelightSubsystem.targetXOffset());
  }
}
