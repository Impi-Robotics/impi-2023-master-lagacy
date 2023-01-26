// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  private Solenoid leftFlapPiston;
  private Solenoid rightFlapPiston;
  private Solenoid leftIntakePiston;
  private Solenoid rightIntakePiston;

  private DigitalInput flapSensor;

  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.CAN.INTAKE_MOTOR,MotorType.kBrushless);
    leftFlapPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.LEFT_FLAP_PISTON);
    rightFlapPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.RIGHT_FLAP_PISTON);
    leftIntakePiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.LEFT_INTAKE_PISTON);
    rightIntakePiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.RIGHT_INTAKE_PISTON);

    flapSensor = new DigitalInput(Constants.DIO.FLAP_SENSOR);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void IntakeJoystick(double speed){
    intakeMotor.set(speed);
  }

  public void IntakeStop() {
    intakeMotor.set(0);
  }

  public void IntakeRun() {
    intakeMotor.set(0.3);
  }

  public void IntakeRunToFlap() {
    if(!getFlapSensor()) {
      intakeMotor.set(0.7);
    } else {
      intakeMotor.set(0);
    }
  }

  public void IntakeExtend() {
    if(!getFlapOpen()) {
      leftIntakePiston.set(true);
      rightIntakePiston.set(true);  
    } else {
      leftIntakePiston.set(false);
      rightIntakePiston.set(false);  
    }
  }

  public void IntakeRetract() {
    leftIntakePiston.set(false);
    rightIntakePiston.set(false);
  }

  public void FlapOpen() {
    if(!getIntakeExtended()) {
      leftFlapPiston.set(true);
      rightFlapPiston.set(true);  
    } else {
      leftFlapPiston.set(false);
      rightFlapPiston.set(false);  
    }
  }

  public void FlapClose() {
    leftFlapPiston.set(false);
    rightFlapPiston.set(false);
  }

  public boolean getIntakeExtended() {
    return leftIntakePiston.get() && rightIntakePiston.get();
  }

  public boolean getFlapOpen() {
    return leftFlapPiston.get() && rightFlapPiston.get();
  }

  public boolean getFlapSensor() {
    return flapSensor.get();
  }
}
