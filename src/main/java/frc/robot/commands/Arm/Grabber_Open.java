package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class Grabber_Open extends InstantCommand {

  private ArmSubsystem armSubsystem;

  public Grabber_Open(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.grabberOpen();
  }
}
