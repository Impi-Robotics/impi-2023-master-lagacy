package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ImpiLib2023;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Arm_Joystick extends CommandBase {
  /** Creates a new ArmJoystick. */

  private ArmSubsystem armSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private DoubleSupplier armJoystick;

  public Arm_Joystick(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DoubleSupplier armJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.armJoystick = armJoystick;
    addRequirements(armSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(armSubsystem.getPassThrough()) {
    //   intakeSubsystem.flapOpen();
    // } else {
    //   intakeSubsystem.flapClose();
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    armSubsystem.armJoystick(ImpiLib2023.deadzone(armJoystick.getAsDouble(), 0.05));
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
