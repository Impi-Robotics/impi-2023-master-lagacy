package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

public class Aligned extends InstantCommand {

  private LEDSubsystem ledSubsystem;

  public Aligned(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.SetLEDsAligned();
  }
}