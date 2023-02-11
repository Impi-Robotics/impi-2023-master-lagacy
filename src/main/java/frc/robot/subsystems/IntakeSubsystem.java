package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private Solenoid flapPiston;
  private Solenoid intakePiston;

  private DigitalInput flapSensor;
  private DigitalInput conveyorSensor;

  public IntakeSubsystem() {
    //motors
    intakeMotor = new CANSparkMax(Constants.CAN.INTAKE_MOTOR,MotorType.kBrushless);
    intakeMotor.setSmartCurrentLimit(Constants.SMART_LIMITS.INTAKE_SMART_LIMIT);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    //intakeMotor.setInverted(true);

    flapPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.FLAP_PISTON);
    intakePiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.INTAKE_PISTON);

    flapSensor = new DigitalInput(Constants.DIO.FLAP_SENSOR);
    conveyorSensor = new DigitalInput(Constants.DIO.CONVEYOR_SENSOR);

    intakeMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void intakeJoystick(double speed) {
    intakeMotor.set(speed);
  }

  public void intakeStop() {
    intakeMotor.set(0);
  }

  public void intakeRun() {
    intakeMotor.set(0.6);
  }

  public boolean getConveyorSensor(){
    return conveyorSensor.get();
  }

  public boolean getFlapSensor() {
    return flapSensor.get();
  }

  public void intakeExtend() {
    intakePiston.set(true);
  }

  public void intakeRetract() {
    intakePiston.set(false);
  }

  public void flapOpen() {
    flapPiston.set(true);
  }

  public void flapClose() {
    flapPiston.set(false);
  }

  //After first sensor sees game piece
  public void intakeRunToFlap() {
    if(!getFlapSensor()){
      intakeRun();
    }
  }

  public boolean isIntakeExtended() {
    return intakePiston.get();
  }

  public boolean isFlapOpen() {
    return flapPiston.get();
  } 
}
