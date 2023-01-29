package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;

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
    ledSubsystem.cubeMode(false);
    ledSubsystem.coneMode(true);
    armSubsystem.cubeMode(ledSubsystem.getCubeMode());
  }
}
