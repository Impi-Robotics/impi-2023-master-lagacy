// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Chassis_AlignmentLimelight extends SequentialCommandGroup {
  private ChassisSubsystem chassisSubsystem;
  private LimelightSubsystem limelightSubsystem;
  /** Creates a new Chassis_AlignmentLimelight. */
  public Chassis_AlignmentLimelight() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Chassis_TurnToTarget(chassisSubsystem),
      new WaitCommand(0.5),
      new Chassis_StrafeToTarget(chassisSubsystem, limelightSubsystem),
      new WaitCommand(0.5), 
      new Chassis_DriveToTarget(chassisSubsystem)
    );
  }
}
