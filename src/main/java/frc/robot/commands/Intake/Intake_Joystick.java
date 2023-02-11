package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ImpiLib2023;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake_Joystick extends CommandBase {
  /** Creates a new Intake_Joystick. */

  private IntakeSubsystem intakeSubsystem;
  private ArmSubsystem armSubsystem;
  private DoubleSupplier reverseSpeed;
  private DoubleSupplier forwardSpeed;

  public Intake_Joystick(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, DoubleSupplier reverseSpeed, DoubleSupplier forwardSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
    this.reverseSpeed = reverseSpeed;
    this.forwardSpeed = forwardSpeed;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeedValue = ImpiLib2023.parseJoystick(forwardSpeed, 0.1);
    double reverseSpeedValue = ImpiLib2023.parseJoystick(reverseSpeed, 0.1);
    if(intakeSubsystem.isFlapOpen()){
      intakeSubsystem.intakeRetract();
      intakeSubsystem.intakeStop();
    }
    else if(intakeSubsystem.getConveyorSensor()){
      intakeSubsystem.intakeRetract();
      intakeSubsystem.intakeRun();
    }
    else if((!intakeSubsystem.getConveyorSensor() && !intakeSubsystem.getFlapSensor()) && !armSubsystem.isGamePieceGrabbed()){
      intakeSubsystem.intakeRun();
    }
    else if(intakeSubsystem.getFlapSensor()){
      //intakeSubsystem.intakeRetract();
      intakeSubsystem.intakeStop();
    }
    //manual override
    else if(Math.abs(forwardSpeedValue) > 0.5 && Math.abs(reverseSpeedValue) > 0.5){
      intakeSubsystem.intakeExtend();
      intakeSubsystem.intakeJoystick(forwardSpeedValue - reverseSpeedValue);
    }
    else{
      intakeSubsystem.intakeRetract();
      intakeSubsystem.intakeStop();
    }
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