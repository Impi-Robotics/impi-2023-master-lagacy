package frc.robot.commands.Arm.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Arm_RunToShelfPosition extends CommandBase {
  /** Creates a new Arm_GoToShelfPosition. */

  private ArmSubsystem armSubsystem;
  private IntakeSubsystem intakeSubsystem;

  public Arm_RunToShelfPosition(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(armSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.armRetract();
    armSubsystem.grabberOpen();
    armSubsystem.flipUp();
    if((armSubsystem.getArmState() == armSubsystem.armStates[2]) || ((armSubsystem.getArmState() == armSubsystem.armStates[6]) && armSubsystem.getMinPassThrough())) {
      intakeSubsystem.flapOpen();      
    } else {
      intakeSubsystem.flapClose();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.goToShelfPosition();
    if(armSubsystem.getPassThrough()) {
      intakeSubsystem.flapOpen();
    } else {
      intakeSubsystem.flapClose();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

