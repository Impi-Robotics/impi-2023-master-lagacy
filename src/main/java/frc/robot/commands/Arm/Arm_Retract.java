package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class Arm_Retract extends InstantCommand {

  public ArmSubsystem armSubsystem;

  public Arm_Retract(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.armRetract();
  }
}
