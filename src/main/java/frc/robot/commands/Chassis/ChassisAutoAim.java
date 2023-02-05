// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class ChassisAutoAim extends CommandBase {
  /** Creates a new ChassisAutoAim. */
  private final ChassisSubsystem chassisSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  public ChassisAutoAim(ChassisSubsystem chassisSubsystem, LimelightSubsystem limelightSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(chassisSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassisSubsystem.setDesiredAngle(limelightSubsystem.targetXOffset());
    chassisSubsystem.drive(0., 0., 0., true, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (chassisSubsystem.isTargetCentered()) {
      return true;
    }
    return false;
  }
}
