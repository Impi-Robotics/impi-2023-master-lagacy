package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CHASSIS;

public class SwerveSubsystem extends SubsystemBase {
  
  private SwerveModule frontLeftSwerve;
  private SwerveModule frontRightSwerve;
  private SwerveModule backLeftSwerve;
  private SwerveModule backRightSwerve;
  
  private AHRS navx;

  // PID Controllers
  private double desiredAngle = 0;
  private PIDController dController;

  private final GenericEntry shuffleboardGyroAngle;
  private final GenericEntry shuffleboardGyroPitch;
  //Odometry... implement last
  public SwerveSubsystem() {

    frontLeftSwerve = new SwerveModule(
      "Front Left",
      Constants.CAN.CHASSIS_FRONT_LEFT_DRIVE_MOTOR,
      Constants.CAN.CHASSIS_FRONT_LEFT_TURN_MOTOR, 
      Constants.AI.CHASSIS_FRONT_LEFT_DRIVE_ENCODER,
      Constants.CHASSIS.SWERVE_FRONT_LEFT_ZERO_ANGLE,
      false
      );
    frontRightSwerve = new SwerveModule(
      "Front RIGHT",
      Constants.CAN.CHASSIS_FRONT_RIGHT_DRIVE_MOTOR,
      Constants.CAN.CHASSIS_FRONT_RIGHT_TURN_MOTOR, 
      Constants.AI.CHASSIS_FRONT_RIGHT_DRIVE_ENCODER,
      Constants.CHASSIS.SWERVE_FRONT_RIGHT_ZERO_ANGLE,
      true
      );
    backLeftSwerve = new SwerveModule(
      "Back Left",
      Constants.CAN.CHASSIS_BACK_LEFT_DRIVE_MOTOR,
      Constants.CAN.CHASSIS_BACK_LEFT_TURN_MOTOR, 
      Constants.AI.CHASSIS_BACK_LEFT_DRIVE_ENCODER,
      Constants.CHASSIS.SWERVE_BACK_LEFT_ZERO_ANGLE,
      false
      );
    backRightSwerve = new SwerveModule(
      "Back RIGHT",
      Constants.CAN.CHASSIS_BACK_RIGHT_DRIVE_MOTOR,
      Constants.CAN.CHASSIS_BACK_RIGHT_TURN_MOTOR, 
      Constants.AI.CHASSIS_BACK_RIGHT_DRIVE_ENCODER,
      Constants.CHASSIS.SWERVE_BACK_RIGHT_ZERO_ANGLE,
      false
      );
      
      dController = new PIDController(0.05, 0., 0.);
      dController.setTolerance(3);
      dController.reset();

      navx = new AHRS(SPI.Port.kMXP);
      new Thread(() -> {
        try {
            Thread.sleep(500);
            navx.reset();
        } catch (Exception e) {
        }
      }).start();

       // Shuffleboard
       ShuffleboardTab tab = Shuffleboard.getTab("Chassis");
       shuffleboardGyroAngle = tab.add("Gyro Angle", 0.)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        shuffleboardGyroPitch = tab.add("Gyro Pitch", 0.)
                .withPosition(1, 1)
                .withSize(1, 1)
                .getEntry();
  }

  @Override
  public void periodic() {
    shuffleboardGyroAngle.setDouble(getGyroAngle());
    shuffleboardGyroPitch.setDouble(navx.getRoll());

    frontLeftSwerve.periodic();
    frontRightSwerve.periodic();
    backLeftSwerve.periodic();
    backRightSwerve.periodic();
  }
  public double getGyroAngle(){
    return navx.getAngle();
  }

  //TEST Methods
  //1)
  /*
   * Method to straighten wheels
   */
  public void straightenWheels(){
    var swerveModuleStates = Constants.CHASSIS.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(1, 0, 0));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.CHASSIS.MAX_METERS_PER_SECOND);
    for(int i = 0; i < 4; i++){
      swerveModuleStates[i].speedMetersPerSecond = 0.;
    }
    frontLeftSwerve.setDesiredState(swerveModuleStates[0]);
    frontRightSwerve.setDesiredState(swerveModuleStates[1]);
    backLeftSwerve.setDesiredState(swerveModuleStates[2]);
    backRightSwerve.setDesiredState(swerveModuleStates[3]);
  }

  //2) Might want to set 0 for x or y speeds to test each direction ind.
  /*
   * Method to drive straight without rotating
   * 
   * @param xSpeed = Speed of robot in x direction... forward
   * @param ySpeed = Speed of robot in y direction... sideways
   */
  public void driveStraight(double xSpeed, double ySpeed){
    var swerveModuleStates = Constants.CHASSIS.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeed * Constants.CHASSIS.MAX_METERS_PER_SECOND, ySpeed * Constants.CHASSIS.MAX_METERS_PER_SECOND, 0));
    
    setDesiredStates(swerveModuleStates);
  }

  //3)
  /*
   * Method to make sure robot is turning as desired
   */
  public void turnToAngle(double desiredAngle){
    double rot = dController.calculate(getGyroAngle(), desiredAngle);
    var swerveModuleStates = CHASSIS.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, 0, rot));
    
    setDesiredStates(swerveModuleStates);
  }
  //4)
  /*
   * Method to drive... no field oriented
   */
  public void dumbDrive(double xSpeed, double ySpeed, double rot){
    dController.enableContinuousInput(-180, 180);
    var swerveModuleStates= CHASSIS.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    setDesiredStates(swerveModuleStates);
  }
  //5)
  /*
   * Drive method.. field oriented
   */
  public void swerveDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
    double currentAngle = getGyroAngle();
    dController.enableContinuousInput(-180, 180);
    var swerveModuleStates = CHASSIS.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                new Rotation2d(currentAngle * Math.PI / 180))
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
    
    setDesiredStates(swerveModuleStates);
  }
  public void setDesiredStates(SwerveModuleState[] swerveModuleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.CHASSIS.MAX_METERS_PER_SECOND);
    frontLeftSwerve.setDesiredState(swerveModuleStates[0]);
    frontRightSwerve.setDesiredState(swerveModuleStates[1]);
    backLeftSwerve.setDesiredState(swerveModuleStates[2]);
    backRightSwerve.setDesiredState(swerveModuleStates[3]);
  }

  public void rotateToTarget(){
    desiredAngle = 0;
    double rot = dController.calculate(getAngle(), desiredAngle);
    dumbDrive(0, 0, rot);
  }

  public void strafeToTarget(double xOffset){
    desiredAngle = 0;
    double rot = dController.calculate(getAngle(), desiredAngle);
    // drive(0, 0, rot, true, false, false);
    if(xOffset > 1 && xOffset < -1) {
      if(xOffset > 1){
        dumbDrive(-1, 0, rot);
      }
      else if(xOffset < -1){
        dumbDrive(1, 0, rot);
      }
      else{
        dumbDrive(0, 0, rot);
      }
    }
  }

  public void setGyroAngle(double angle) {
    desiredAngle = angle;
  }

  public double getAngle() {
    return -navx.getAngle();
  }

  public void updateGyro() {
    getAngle();
  }

  public void stop(){
    frontLeftSwerve.stop();
    frontRightSwerve.stop();
    backLeftSwerve.stop();
    backRightSwerve.stop();
  }

  public boolean isTargetCentered(double xOffset) {
    if(xOffset < 1 && xOffset > -1) {
      return true;
    } else {
      return false;
    }
  }
  
  public double setDesiredAngle(double desiredAngle) {
    return desiredAngle;
  }
}
