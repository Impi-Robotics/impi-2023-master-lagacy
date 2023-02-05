package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
//import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.SWERVE;
import frc.robot.Constants.Swerve;

public class ChassisSubsystem extends SubsystemBase {
    // Sensors
    private AHRS gyro;

    // Swerve Modules
    private SwerveModule swerveFrontRight;
    private SwerveModule swerveBackRight;
    private SwerveModule swerveFrontLeft;
    private SwerveModule swerveBackLeft;

    // PID Controllers
    private PIDController directionController;
    private double desiredAngle = 0;

    // Odometry
    private SwerveDriveOdometry swerveDriveOdometry;
    private double targetXCoordinate = -4.13;
    private double targetYCoordinate = 1.42;
    private double currentXCoordinate;
    private double currentYCoordinate;
    private double displacementXValue;
    private double displacementYValue;
    private double angleToTargetValue;
    private boolean turnToTarget = false;
    // private boolean manualMode = true;

    // Shuffleboard
    // private final NetworkTableEntry shuffleboardPoseX;
    // private final NetworkTableEntry shuffleboardPoseY;
    // private final NetworkTableEntry shuffleboardGyroAngle;
    // private final NetworkTableEntry shuffleboardGyroPitch;
    // private final NetworkTableEntry shuffleboarDesiredAngle;

    public ChassisSubsystem() {
        // Sensors
        gyro = new AHRS(SPI.Port.kMXP);
        //gyro = new AHRS(Port.kUSB1);
        // gyro;
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.reset();
            } catch (Exception e) {
            }
        }).start();

        // Swerve Modules
        swerveFrontLeft = new SwerveModule(
                "Front Left",
                Constants.CAN.CHASSIS_FRONT_LEFT_DRIVE_MOTOR,
                Constants.CAN.CHASSIS_FRONT_LEFT_TURN_MOTOR,
                Constants.AI.FRONT_LEFT_DRIVE_ENCODER,
                Constants.SWERVE.FRONT_LEFT_ZERO_ANGLE);
        swerveFrontRight = new SwerveModule(
                "Front Right",
                Constants.CAN.CHASSIS_FRONT_RIGHT_DRIVE_MOTOR,
                Constants.CAN.CHASSIS_FRONT_RIGHT_TURN_MOTOR,
                Constants.AI.FRONT_RIGHT_DRIVE_ENCODER,
                Constants.SWERVE.FRONT_RIGHT_ZERO_ANGLE);

        swerveBackLeft = new SwerveModule(
                "Back Left",
                Constants.CAN.CHASSIS_REAR_LEFT_DRIVE_MOTOR,
                Constants.CAN.CHASSIS_REAR_LEFT_TURN_MOTOR,
                Constants.AI.REAR_LEFT_DRIVE_ENCODER,
                Constants.SWERVE.REAR_LEFT_ZERO_ANGLE);

        swerveBackRight = new SwerveModule(
                "Back Right",
                Constants.CAN.CHASSIS_REAR_RIGHT_DRIVE_MOTOR,
                Constants.CAN.CHASSIS_REAR_RIGHT_TURN_MOTOR,
                Constants.AI.REAR_RIGHT_DRIVE_ENCODER,
                Constants.SWERVE.REAR_RIGHT_ZERO_ANGLE);

        // PID Controllers
        //directionController = new PIDController(0.1, 0.0, 0.015);
        directionController = new PIDController(0.045, 0.0075, 0.00005);
        directionController.setTolerance(10.);
        directionController.reset();

        // currentXCoordinate = getPose().getX();
        // currentYCoordinate = getPose().getY();
        // displacementXValue = currentXCoordinate - targetXCoordinate;

        // Odometry
        // swerveDriveOdometry = new SwerveDriveOdometry(Constants.SWERVE.DRIVE_KINEMATICS, new Rotation2d(0));

        // Shuffleboard
        // ShuffleboardTab tab = Shuffleboard.getTab("Chassis");
        // shuffleboardPoseX = tab.add("Pose X", 0.)
        //         .withPosition(0, 0)
        //         .withSize(1, 1)
        //         .getEntry();
        // shuffleboardPoseY = tab.add("Pose Y", 0.)
        //         .withPosition(1, 0)
        //         .withSize(1, 1)
        //         .getEntry();
        // shuffleboardGyroAngle = tab.add("Gyro Angle", 0.)
        //         .withPosition(0, 1)
        //         .withSize(1, 1)
        //         .getEntry();
        // shuffleboardGyroPitch = tab.add("Gyro Pitch", 0.)
        //         .withPosition(1, 1)
        //         .withSize(1, 1)
        //         .getEntry();
        // shuffleboarDesiredAngle = tab.add("Desired Angle", 0.)
        //         .withPosition(2, 0)
        //         .withSize(1, 1)
        //         .getEntry();
    }

    @Override
    public void periodic() {
        // // Odometry - take off Shuffleboard when updated with correct values for comp.
        // swerveDriveOdometry.update(Rotation2d.fromDegrees(getAngle()), swerveFrontLeft.getState(),
        //         swerveFrontRight.getState(), swerveBackLeft.getState(), swerveBackRight.getState());

        // // Shuffleboard
        // shuffleboardPoseX.setDouble(getPose().getX());
        // shuffleboardPoseY.setDouble(getPose().getY());
        // shuffleboardGyroAngle.setDouble(getGyroAngle());
        // shuffleboardGyroPitch.setDouble(gyro.getRoll());
        // shuffleboarDesiredAngle.setDouble(desiredAngle);

        swerveFrontLeft.periodic();
        swerveFrontRight.periodic();
        swerveBackLeft.periodic();
        swerveBackRight.periodic();
        
        SmartDashboard.putData(directionController);
/*
        double Kp = SmartDashboard.getNumber("Turn Kp", directionController.getP());
        SmartDashboard.putNumber("Turn Kp", Kp);
        double Ki = SmartDashboard.getNumber("Turn Ki", directionController.getI());
        SmartDashboard.putNumber("Turn Ki", Ki);
        double Kd = SmartDashboard.getNumber("Turn Kd", directionController.getD());
        SmartDashboard.putNumber("Turn Kd", Kd);
         
        directionController.setP(Kp);
        directionController.setI(Ki);
        directionController.setD(Kd);
         */

    }

    /*
     * Method to drive the robot without rotating
     *
     * @param xSpeed Speed of the robot in the xDirection (forward)
     * @param ySpeed Speed of the robot in the yDirection (sideways)
     */
    public void straightenWheels() {
        var swerveModuleStates = SWERVE.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(1.0, 0, 0));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, CHASSIS.MAX_METERS_PER_SECOND);
        for (int ii=0; ii<4; ii++) {
            swerveModuleStates[ii].speedMetersPerSecond = 0.;
        }

        swerveFrontLeft.setDesiredState(swerveModuleStates[1]);
        swerveFrontRight.setDesiredState(swerveModuleStates[0]);
        swerveBackLeft.setDesiredState(swerveModuleStates[3]);
        swerveBackRight.setDesiredState(swerveModuleStates[2]);
    }

    public void driveStraight(double xSpeed, double ySpeed) {
        var swerveModuleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, 0));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, CHASSIS.MAX_METERS_PER_SECOND);

        swerveFrontLeft.setDesiredState(swerveModuleStates[1]);
        swerveFrontRight.setDesiredState(swerveModuleStates[0]);
        swerveBackLeft.setDesiredState(swerveModuleStates[3]);
        swerveBackRight.setDesiredState(swerveModuleStates[2]);
    }
    public void turnToAngle(double desiredAngle){
    
        setDesiredAngle(desiredAngle);
        double rot = directionController.calculate(getAngle(), desiredAngle);
        drive(0, 0, rot, true, false, false);
    }
    public void setPid(double p , double i, double d){
        directionController.setP(p);
        directionController.setI(i);
        directionController.setD(d);
    }

    public void resetDriveEncoderPositions() {
        swerveFrontLeft.resetDriveEncoderPosition();
        swerveFrontRight.resetDriveEncoderPosition();
        swerveBackLeft.resetDriveEncoderPosition();
        swerveBackRight.resetDriveEncoderPosition();
    }

    public double getDriveStraightEncoderPosition() {
        return swerveFrontLeft.getDriveEncoderPosition();
    }

    /*
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * 
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     * field.
     * 
     * @param trackTargetVision Whether rotation is determined by limelight relative
     * to current gyro angle
     * 
     * @param trackTargetOdometry Whether we want robot to face in relative
     * direction of target based off of robot position
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean trackTarget, boolean rotateDirection) {

        //double currentAngle = gyro.getAngle();
        double currentAngle = getGyroAngle();
        // VISION TRACKING: PID calculates rotation value based on current robot angle
        // and xOffset from limelight
        if (trackTarget) {
            rot = -directionController.calculate(-currentAngle, -currentAngle + desiredAngle);
        } else if (rotateDirection) {
            rot = directionController.calculate(-currentAngle, desiredAngle);
        }
        // ODOMETRY: Calculates desired angle based off of robot position on field
        /*
        if (getTurnToTargetOdometry()) {
            desiredAngle = -(180 - angleToTargetValue);
            rot = -directionController.calculate(-currentAngle, desiredAngle);
        }
        */
        // DRIVE:
        directionController.enableContinuousInput(-180, 180);
        var swerveModuleStates = SWERVE.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                new Rotation2d(currentAngle * Math.PI / 180))
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        /*
        var swerveModuleStates = SWERVE.DRIVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                        new Rotation2d(currentAngle * Math.PI / 180)));
        */
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, CHASSIS.MAX_METERS_PER_SECOND);

        swerveFrontLeft.setDesiredState(swerveModuleStates[1]);
        swerveFrontRight.setDesiredState(swerveModuleStates[0]);
        swerveBackLeft.setDesiredState(swerveModuleStates[3]);
        swerveBackRight.setDesiredState(swerveModuleStates[2]);
    }

    public void rotateToTarget(){
      desiredAngle = 0;
      double rot = directionController.calculate(getAngle(), desiredAngle);
      drive(0, 0, rot, true, false, false);
    }

    public void strafeToTarget(double xOffset){
      desiredAngle = 0;
      double rot = directionController.calculate(getAngle(), desiredAngle);
      // drive(0, 0, rot, true, false, false);
      if(xOffset > 1 && xOffset < -1) {
        if(xOffset > 1){
          drive(-1, 0, 0, false, false, false);
        }
        else if(xOffset < -1){
          drive(1, 0, 0, false, false, false);
        }
        else{
          drive(0, 0, 0, false, false, false);
        }
      }
    }

    // Turns robot 10 degrees to the right
    public void rotateRightFixedAngle() {
        desiredAngle -= 10;
    }

    // Turns robot so intake is facing 90 degrees to the right
    public void rotateFieldRight() {
        desiredAngle = -90;
        double rot = directionController.calculate(getAngle(), desiredAngle);
        drive(0, 0, rot, true, false, false);
    }

    public double getGyroAngle(){
        return gyro.getAngle();
    }

    public double getGyroYaw(){
      return gyro.getYaw();
  }

    // Turns the robot so intake is facing 90 degrees to the left
    public void rotateFieldLeft() {
        desiredAngle = 90;
        double rot = directionController.calculate(getAngle(), desiredAngle);
        drive(0, 0, rot, true, false, false);
    }

    // Turns the robot so intake is facing forwards
    public void rotateFieldForward() {
        desiredAngle = 0;
        double rot = directionController.calculate(getAngle(), desiredAngle);
        drive(0, 0, rot, true, false, false);
    }

    // Turns the robot so intake is facing backwards
    public void rotateFieldBackward() {
        desiredAngle = -180;
        double rot = directionController.calculate(getAngle(), desiredAngle);
        drive(0, 0, rot, true, false, false);
    }

    // Sets desired angle for PID to calculate
    public void setDesiredAngle(double angle) {
        desiredAngle = angle;
    }

    // Turning based off PID - we don't use this anymore
    public void incrementDesiredAngle(double angle) {
        desiredAngle += angle * 10;
    }

    public void driveManual(double drive, double turn) {
        swerveFrontRight.driveManual(drive, turn);
        swerveBackRight.driveManual(drive, turn);
        swerveFrontLeft.driveManual(drive, turn);
        swerveBackLeft.driveManual(drive, turn);
    }

    public void driveTest(double speed, double angleX, double angleY) {
        swerveFrontRight.testPIDControl(speed, angleX, angleY);
        swerveBackRight.testPIDControl(speed, angleX, angleY);
        swerveFrontLeft.testPIDControl(speed, angleX, angleY);
        swerveBackLeft.testPIDControl(speed, angleX, angleY);
    }

    /*
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, CHASSIS.MAX_METERS_PER_SECOND);

        swerveFrontLeft.setDesiredState(desiredStates[0]);
        swerveFrontRight.setDesiredState(desiredStates[1]);
        swerveBackLeft.setDesiredState(desiredStates[2]);
        swerveBackRight.setDesiredState(desiredStates[3]);
    }

    public boolean getTurnToTargetOdometry() {
        return turnToTarget;
    }

    public void setGyroAngle(double angle) {
        gyro.reset();
        gyro.setAngleAdjustment(angle);
    }

    public boolean isTargetCentered() {
        if (Math.abs(desiredAngle) < 1) {
            return true;
        }
        return false;
    }
    public void setBrakeMode(){
        swerveBackLeft.setBrakeMode();
        swerveBackRight.setBrakeMode();
        swerveFrontLeft.setBrakeMode();
        swerveFrontRight.setBrakeMode();
    }
    public void setCoastMode(){
        swerveBackLeft.setCoastMode();
        swerveBackRight.setCoastMode();
        swerveFrontLeft.setCoastMode();
        swerveFrontRight.setCoastMode();
    }

    // Resets gyro and resets odometry position
    public void resetGyro() {
        gyro.reset();
        gyro.resetDisplacement();
        desiredAngle = 0;
        // Change position once we figure out where we want to reset during match
    }

    public double getAngle() {
        return -gyro.getAngle();
    }

    // Get's pitch of robot from NAV X
    public double getPitch() {
        return gyro.getRoll();
    }

    public void updateGyro() {
        gyro.getAngle();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public Pose2d getPose() {
        return swerveDriveOdometry.getPoseMeters();
    }

    // public void resetOdometry(Pose2d pose) {
    //     resetGyro();
    //     swerveDriveOdometry.resetPosition(pose, getRotation2d());
    // }

    // Toggle so we can turn odometry on and off during match
    public void turnToTargetOdometry() {
        if (turnToTarget) {
            turnToTarget = false;
        } else {
            turnToTarget = true;
        }
    }

    public void turnToTargetOdometryOff() {
        turnToTarget = false;
    }

    // Stops swerve modules
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    public void setSwerveModuleP(double p){
        swerveFrontRight.setP(p);
        swerveBackRight.setP(p);
        swerveFrontLeft.setP(p);
        swerveBackLeft.setP(p);
    }

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public void stop() {
        swerveFrontRight.driveManual(0., 0.);
        swerveBackRight.driveManual(0., 0.);
        swerveFrontLeft.driveManual(0., 0.);
        swerveBackLeft.driveManual(0., 0.);
    }
}