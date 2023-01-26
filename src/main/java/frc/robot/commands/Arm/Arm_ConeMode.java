// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Arm_ConeMode extends InstantCommand {
  private ArmSubsystem armSubsystem;
  private LEDSubsystem ledSubsystem;
  public Arm_ConeMode(ArmSubsystem armSubsystem, LEDSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.ledSubsystem = ledSubsystem;
    addRequirements(armSubsystem);
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.CubeMode(false);
    ledSubsystem.ConeMode(true);
    armSubsystem.CubeMode(ledSubsystem.getCubeMode());
  }
}
