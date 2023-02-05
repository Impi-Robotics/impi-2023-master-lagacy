package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax intakeMotor;
  private Solenoid flapPiston;
  private Solenoid intakePiston;

  private DigitalInput flapSensor;

  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.CAN.INTAKE_MOTOR,MotorType.kBrushless);
    flapPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.FLAP_PISTON);
    intakePiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.INTAKE_PISTON);

    flapSensor = new DigitalInput(Constants.DIO.FLAP_SENSOR);
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
    intakeMotor.set(0.3);
  }

  public void intakeRunToFlap() {
    if(!getFlapSensor()) {
      intakeMotor.set(0.7);
    } else {
      intakeMotor.set(0);
    }
  }

  public void intakeExtend() {
    if(!getFlapOpen()) {
      intakePiston.set(true);
    } else {
      intakePiston.set(false);
    }
  }

  public void intakeRetract() {
    intakePiston.set(false);
  }

  public void flapOpen() {
    if(!getIntakeExtended()) {
      flapPiston.set(true);
    } else {
      flapPiston.set(false);
    }
  }

  public void flapClose() {
    flapPiston.set(false);
  }

  public boolean getIntakeExtended() {
    return intakePiston.get();
  }

  public boolean getFlapOpen() {
    return flapPiston.get();
  }

  public boolean getFlapSensor() {
    return flapSensor.get();
  }
}
