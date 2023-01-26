// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  // private LEDSubsystem ledSubsystem;

  private Solenoid leftArmPiston;
  private Solenoid rightArmPiston;
  private Solenoid leftGrabberPiston;
  private Solenoid rightGrabberPiston;
  private Solenoid flipPiston;
  private CANSparkMax leftArmMotor;
  private CANSparkMax rightArmMotor;

  private SparkMaxPIDController armPIDController;
  private RelativeEncoder leftArmEncoder;
  // private DigitalInput flapSensor;
  private DigitalInput armLimitSwitch;

  private double setpoint;
  //Cone/cube mode == incorporate LED's. Purple = cube, yellow = cone
  //When purple subtract x amount from setpoint
  private boolean cubeMode = false;
  //Maybe get rid of
  // private boolean softStopHit = false;
  //Debugging: shows what position arm is supposed to be in
  public String[] armStates = {"Floor", "Shelf", "Conveyor", "Low", "Medium", "High"};
  //Shelf, Drive, Low, Medium, High, Floor
  //Current state
  private String armState;

  final GenericEntry shuffleboardArmKP;
  final GenericEntry shuffleboardArmKI;
  final GenericEntry shuffleboardArmKD;
  final GenericEntry shuffleboardArmKFF;
  final GenericEntry shuffleboardArmBase;
  final GenericEntry shuffleboardArmState;
  final GenericEntry shuffleboardArmEncoder;
  final GenericEntry shuffleboardSetpoint;
  final GenericEntry shuffleboardGrabberState;
  final GenericEntry shuffleboardGrabberFlip;
  final GenericEntry shuffleboardCubeMode;
  final GenericEntry shuffleboardArmHardStop;
  final GenericEntry shuffleboardArmSoftStop;


  public ArmSubsystem() {

    // arm extension pistons
    leftArmPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.LEFT_ARM_PISTON);
    rightArmPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.RIGHT_ARM_PISTON);

    // grabber pistons
    leftGrabberPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.LEFT_GRABBER_PISTON);
    rightGrabberPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.RIGHT_GRABBER_PISTON);
    flipPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.FLIP_GRABBER_PISTON);

    // arm maneuvering motors
    leftArmMotor = new CANSparkMax(Constants.CAN.LEFT_ARM_MOTOR, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(Constants.CAN.RIGHT_ARM_MOTOR, MotorType.kBrushless);

    // arm encoders
    leftArmEncoder = leftArmMotor.getEncoder();
    // sensors
    // flapSensor = new DigitalInput(Constants.DIO.FLAP_SENSOR);
    armLimitSwitch = new DigitalInput(Constants.DIO.LIMIT_SWITCH);

    // arm position presets
    //
    // double setpoint;
    // double nodeAdjust = 50;
    // double posFloor = 0;
    // double posConveyor = 0;
    // double posHP = 0;
    // double posNode1 = 0;
    // double posNode2 = 0;
    // double posNode3 = 0;

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.follow(leftArmMotor, true);

    // arm pid
    armPIDController = leftArmMotor.getPIDController();
    //Get rid of K in constants
    armPIDController.setP(Constants.ARM.P);
    armPIDController.setI(Constants.ARM.I);
    armPIDController.setD(Constants.ARM.D);
    armPIDController.setFF(Constants.ARM.FF);

    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
    // ShuffleboardTab grabberTab = Shuffleboard.getTab("Grabber");

    shuffleboardArmKP = armTab.add("Arm P:", 0.)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardArmKI = armTab.add("Arm I:", 0.)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardArmKD = armTab.add("Arm D:", 0.)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardArmKFF = armTab.add("Arm FF:", 0.)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardArmBase = armTab.add("Arm Base:", false)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardArmState = armTab.add("Arm State:", false)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardArmEncoder = armTab.add("Arm Encoder:", 0.)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardSetpoint = armTab.add("Arm Setpoint:", 0.)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardGrabberState = armTab.add("Grabber Closed:", false)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardGrabberFlip = armTab.add("Grabber Down:", true)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardCubeMode = armTab.add("Node Switch:", false)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardArmHardStop = armTab.add("Arm Hard Stop:", false)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardArmSoftStop = armTab.add("Arm Soft Stop:", false)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Telemetry - SmartDashboard
    // P
    shuffleboardArmKP.setDouble(getArmP());
    // I
    shuffleboardArmKI.setDouble(getArmI());
    // D
    shuffleboardArmKD.setDouble(getArmD());
    // FF
    shuffleboardArmKFF.setDouble(getArmFF());
    // arm retracted/extended
    shuffleboardArmBase.setBoolean(getArmExtended());
    // arm pos state
    shuffleboardArmState.setString(getArmState());
    // arm pos encoder
    shuffleboardArmEncoder.setDouble(leftArmEncoder.getPosition());
    // arm pos setpoint
    shuffleboardSetpoint.setDouble(getSetpoint());
    // grabber closed/open
    shuffleboardGrabberState.setBoolean(getGrabberClosed());
    // grabber up/down
    shuffleboardGrabberFlip.setBoolean(flipPiston.get());
    // node switch
    shuffleboardCubeMode.setBoolean(cubeMode);
    // arm hard stop hit
    shuffleboardArmHardStop.setBoolean(armLimitSwitch.get());
    // arm soft stop hit
    // shuffleboardArmSoftStop.setBoolean(softStopHit);
  }

  // arm stop
  public void ArmStop() {
    leftArmMotor.set(0.);
  }

  // arm extension
  public void ArmExtend() {
    leftArmPiston.set(true);
    rightArmPiston.set(true);
  }

  // arm retraction
  public void ArmRetract() {
    leftArmPiston.set(false);
    rightArmPiston.set(false);
  }

  // arm piston toggle
  // public void ArmToggle() {
  // if(!leftArmPiston.get()) {
  // ArmExtend();
  // } else {
  // ArmRetract();
  // }
  // }

  // grabber retraction
  public void GrabberClose() {
    leftGrabberPiston.set(true);
    rightGrabberPiston.set(true);
  }

  // grabber extension
  public void GrabberOpen() {
    leftGrabberPiston.set(false);
    rightGrabberPiston.set(false);
  }

  // grabber claw position toggle
  // public void Grabber_Toggle() {
  // if(!leftGrabberPiston.get()) {
  // GrabberClose();
  // } else {
  // GrabberOpen();
  // }
  // }

  // grabber flip lift
  public void GrabberUp() {
      flipPiston.set(true);
  }

  // grabber flip shut
  public void GrabberDown() {
      flipPiston.set(false);
  }

  // grabber claw position toggle
  // public void GrabberLatchToggle() {
  // if(!flipPiston.get()) {
  // GrabberUp();
  // } else {
  // GrabberDown();
  // }
  // }

  // arm joystick input
  public void ArmJoystick(double speed) {
    // can't reach past hard stop
    if (armLimitSwitch.get()) {
      if (speed > 0) {
        speed = 0;
      }
    }
    leftArmMotor.set(speed);
  }
  //public void RunToPosition(double setpoint)
  // arm to position
  public void RunToPosition(double setpoint, String pos) {
    this.setpoint = setpoint;
    armPIDController.setReference(setpoint, ControlType.kPosition);
    armState = pos;
    // pid to position
  }

  public void GoToFloorPosition(){
    setpoint = Constants.ARM.FLOOR_POSITION;
    armState = armStates[0];
    RunToPosition(setpoint, armState);
  }

  public void GoToShelfPosition(){
    setpoint = Constants.ARM.SHELF_POSITION;
    armState = armStates[1];
    RunToPosition(setpoint, armState);
  }
  
  public void GoToDrivePosition(){
    setpoint = Constants.ARM.DRIVE_POSITION;
    armState = armStates[2];
    RunToPosition(setpoint, armState);
  }

  public void GoToLowNode(){
    if(cubeMode){
      setpoint -= Constants.ARM.CUBE_ADJUST;
    }
    setpoint = Constants.ARM.LOW_NODE;
    armState = armStates[3];
    RunToPosition(setpoint, armState);
  }

  public void GoToMediumNode(){
    if(cubeMode){
      setpoint -= Constants.ARM.CUBE_ADJUST;
    }
    setpoint = Constants.ARM.MEDIUM_NODE;
    armState = armStates[4];
    RunToPosition(setpoint, armState);
  }

  public void GoToHighNode(){
    if(cubeMode){
      setpoint -= Constants.ARM.CUBE_ADJUST;
    }
    setpoint = Constants.ARM.HIGH_NODE;
    armState = armStates[5];
    RunToPosition(setpoint, armState);
  }

  public void CubeMode(boolean cubeMode) {
    this.cubeMode = cubeMode;
  }

  //Put in LED... cone mode and cube mode... need to see it from LED subsystem
  // public void ArmCubeMode() {
  //   cubeMode = true;
  // }

  // public void ArmConeMode() {
  //   cubeMode = false;
  // }

  public double getArmP() {
    return armPIDController.getP();
  }

  public double getArmI() {
    return armPIDController.getI();
  }

  public double getArmD() {
    return armPIDController.getD();
  }

  public double getArmFF() {
    return armPIDController.getFF();
  }

  public String getArmState() {
    return armState;
  }

  public boolean getArmExtended() {
    return leftArmPiston.get() && rightArmPiston.get();
  }

  public boolean getGrabberClosed() {
    return leftGrabberPiston.get() && rightGrabberPiston.get();
  }

  public double getSetpoint() {
    return setpoint;
  }

  // public boolean getCubeMode() {
  //   return ledSubsystem.getCubeMode();
  // }

  // public boolean getFlapSensor() {
  //   return flapSensor.get();
  // }
}
