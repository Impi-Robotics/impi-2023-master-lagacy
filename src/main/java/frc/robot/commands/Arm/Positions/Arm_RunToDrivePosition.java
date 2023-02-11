package frc.robot.commands.Arm.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Arm_RunToDrivePosition extends CommandBase {
  /** Creates a new Arm_GoToConveyorPosition. */

  private ArmSubsystem armSubsystem;
  private IntakeSubsystem intakeSubsystem;

  public Arm_RunToDrivePosition(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(armSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.armRetract();
    //armSubsystem.grabberOpen();
    intakeSubsystem.flapOpen();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(armSubsystem.getPassThrough()) {
      intakeSubsystem.flapOpen();
      armSubsystem.goToDrivePosition();
    } else {
      intakeSubsystem.flapClose();
      armSubsystem.goToDrivePosition();
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