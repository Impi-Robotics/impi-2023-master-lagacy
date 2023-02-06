package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private Solenoid armPiston;
  private Solenoid grabberPiston;
  private Solenoid flipPiston;
  private CANSparkMax leftArmMotor;
  private CANSparkMax rightArmMotor;

  private SparkMaxPIDController armPIDController;

  private RelativeEncoder leftArmEncoder;

  private DigitalInput flipSensor;

  private double setpoint;
  //Cone/cube mode == incorporate LED's. Purple = cube, yellow = cone
  //When purple subtract x amount from setpoint
  private boolean cubeMode = false;
  //Debugging: shows what position arm is supposed to be in
  public String[] armStates = {"Floor", "Shelf", "Drive", "Low", "Medium", "High", "Joystick"};
  //Current state
  private String armState;

  // private final GenericEntry shuffleboardArmP;
  // private final GenericEntry shuffleboardArmI;
  // private final GenericEntry shuffleboardArmD;
  // private final GenericEntry shuffleboardArmFF;
  private final GenericEntry shuffleboardArmBase;
  private final GenericEntry shuffleboardArmState;
  private final GenericEntry shuffleboardArmEncoder;
  private final GenericEntry shuffleboardSetpoint;
  private final GenericEntry shuffleboardGrabberState;
  private final GenericEntry shuffleboardGrabberFlip;
  private final GenericEntry shuffleboardFlipSensor;
  private final GenericEntry shuffleboardCubeMode;

  public ArmSubsystem() {
    
    // arm extension pistons
    armPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.ARM_PISTON);

    // grabber pistons
    grabberPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.GRABBER_PISTON);
    flipPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PCM.FLIP_GRABBER_PISTON);

    // arm maneuvering motors
    leftArmMotor = new CANSparkMax(Constants.CAN.LEFT_ARM_MOTOR, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(Constants.CAN.RIGHT_ARM_MOTOR, MotorType.kBrushless);

    // arm encoders
    leftArmEncoder = leftArmMotor.getEncoder();

    // sensors
    flipSensor = new DigitalInput(Constants.DIO.FLIP_SENSOR);
    leftArmMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.ARM.SOFT_STOP_REVERSE);
    leftArmMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ARM.SOFT_STOP_FORWARD);

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.follow(leftArmMotor, true);

    // arm pid
    // armPIDController = leftArmMotor.getPIDController();
    // armPIDController.setP(Constants.ARM.P);
    // armPIDController.setI(Constants.ARM.I);
    // armPIDController.setD(Constants.ARM.D);
    // armPIDController.setFF(Constants.ARM.FF);

    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    // shuffleboardArmP = armTab.add("Arm P:", Constants.ARM.P)
    //     .withPosition(0, 0)
    //     .withSize(1, 1)
    //     .getEntry();
    // shuffleboardArmI = armTab.add("Arm I:", Constants.ARM.I)
    //     .withPosition(0, 0)
    //     .withSize(1, 1)
    //     .getEntry();
    // shuffleboardArmD = armTab.add("Arm D:", Constants.ARM.D)
    //     .withPosition(0, 0)
    //     .withSize(1, 1)
    //     .getEntry();
    // shuffleboardArmFF = armTab.add("Arm FF:", Constants.ARM.FF)
    //     .withPosition(0, 0)
    //     .withSize(1, 1)
    //     .getEntry();
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
    shuffleboardFlipSensor = armTab.add("Holding:", false)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
    shuffleboardCubeMode = armTab.add("Node Switch:", false)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Telemetry - SmartDashboard
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
    // grabber holding
    shuffleboardFlipSensor.setBoolean(getFlipSensor());
    // node switch
    shuffleboardCubeMode.setBoolean(cubeMode);
  }

  // arm joystick input
  public void armJoystick(double speed) {
    // // can't reach past hard stop
    // if (armLimitSwitch.get()) {
    //   if (speed > 0) {
    //     speed = 0;
    //   }
    // }
    armState = armStates[6];
    leftArmMotor.set(speed);
  }

  // arm to position
  public void runToPosition(double setpoint) {
    // this.setpoint = setpoint;
    // armPIDController.setReference(setpoint, ControlType.kPosition);
  }

  public void goToFloorPosition() {
    setpoint = Constants.ARM.FLOOR_POSITION;
    armState = armStates[0];
    runToPosition(setpoint);
  }

  public void goToShelfPosition() {
    setpoint = Constants.ARM.SHELF_POSITION;
    armState = armStates[1];
    runToPosition(setpoint);
  }
  
  public void goToDrivePosition() {
    setpoint = Constants.ARM.DRIVE_POSITION;
    armState = armStates[2];
    runToPosition(setpoint);
  }

  public void goToLowNode() {
    setpoint = Constants.ARM.LOW_NODE;
    if(cubeMode){
      setpoint -= Constants.ARM.CUBE_ADJUST;
    }
    armState = armStates[3];
    runToPosition(setpoint);
  }

  public void goToMediumNode() {
    setpoint = Constants.ARM.MEDIUM_NODE;
    if(cubeMode){
      setpoint -= Constants.ARM.CUBE_ADJUST;
    }
    armState = armStates[4];
    runToPosition(setpoint);
  }

  public void goToHighNode() {
    setpoint = Constants.ARM.HIGH_NODE;
    if(cubeMode){
      setpoint -= Constants.ARM.CUBE_ADJUST;
    }
    armState = armStates[5];
    runToPosition(setpoint);
  }

  // arm stop
  public void armStop() {
    leftArmMotor.set(0.);
  }

  // arm extension
  public void armExtend() {
    armPiston.set(true);
  }

  // arm retraction
  public void armRetract() {
    armPiston.set(false);
  }

  // grabber retraction
  public void grabberClose() {
    grabberPiston.set(true);
  }

  // grabber extension
  public void grabberOpen() {
    grabberPiston.set(false);
  }

  // grabber flip lift
  public void flipUp() {
      flipPiston.set(true);
  }

  // grabber flip shut
  public void flipDown() {
      flipPiston.set(false);
  }

  public void cubeMode(boolean cubeMode) {
    this.cubeMode = cubeMode;
  }

  public boolean getPassThrough() {
    return (getArmEncoder() > Constants.ARM.MIN_PASS_THROUGH) && (getArmEncoder() < Constants.ARM.MAX_PASS_THROUGH);
  }

  public boolean getMinPassThrough() {
    return getArmEncoder() < Constants.ARM.MIN_PASS_THROUGH;
  }

  public boolean getMaxPassThrough() {
    return getArmEncoder() > Constants.ARM.MAX_PASS_THROUGH;
  }

  public double getArmEncoder() {
    return leftArmEncoder.getPosition();
  }

  public String getArmState() {
    return armState;
  }

  public boolean getArmExtended() {
    return armPiston.get();
  }

  public boolean getGrabberClosed() {
    return grabberPiston.get();
  }

  public boolean getFlipSensor() {
    return flipSensor.get();
  }

  public double getSetpoint() {
    return setpoint;
  }

  // public boolean getCubeMode() {
  //   return ledSubsystem.getCubeMode();
  // }
}
