// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeLoading extends InstantCommand {

  private LEDSubsystem ledSubsystem;

  public CubeLoading(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.setCubeLoading();
    ledSubsystem.setLEDsCubeLoading();
  }
}
