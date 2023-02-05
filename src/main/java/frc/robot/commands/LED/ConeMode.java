package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

public class ConeMode extends InstantCommand {

  private LEDSubsystem ledSubsystem;

  public ConeMode() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.SetLEDsConeMode();
  }
}
