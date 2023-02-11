package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class Arm_CubeMode extends InstantCommand {

  private ArmSubsystem armSubsystem;
  private LEDSubsystem ledSubsystem;

  public Arm_CubeMode(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.cubeMode(ledSubsystem.getCubeMode());
  }
}
