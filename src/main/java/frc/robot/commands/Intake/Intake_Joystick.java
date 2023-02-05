package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ImpiLib2023;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake_Joystick extends CommandBase {
  /** Creates a new Intake_Joystick. */

  private IntakeSubsystem intakeSubsystem;
  private DoubleSupplier reverseSpeed;
  private DoubleSupplier forwardSpeed;

  public Intake_Joystick(IntakeSubsystem intakeSubsystem, DoubleSupplier reverseSpeed, DoubleSupplier forwardSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.reverseSpeed = reverseSpeed;
    this.forwardSpeed = forwardSpeed;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.intakeJoystick(ImpiLib2023.deadzone(forwardSpeed.getAsDouble(), 0.1) - ImpiLib2023.deadzone(reverseSpeed.getAsDouble(), 0.1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
