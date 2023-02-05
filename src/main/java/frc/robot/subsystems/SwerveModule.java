package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {

    // Motor Controllers
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    // Sensors
    private RelativeEncoder driveEncoder;
    private AnalogInput turnEncoder;

    // PID Controller
    private final ProfiledPIDController drivePIDController = new ProfiledPIDController(
        Constants.CHASSIS.DRIVE_P, 
        Constants.CHASSIS.DRIVE_I, 
        Constants.CHASSIS.DRIVE_D,
        new TrapezoidProfile.Constraints(
            Constants.CHASSIS.MAX_METERS_PER_SECOND,
            Constants.CHASSIS.MAX_METERS_PER_SECOND_SQUARED));
    private final PIDController turningPIDController;

    // Other
    private double turnZeroAngle;
    private String moduleName;

    /*
     * @param moduleName Gives each module a name
     * 
     * @param driveMotorCANChannel Sets drive motor controller CAN channel
     * 
     * @param turnMotorCANChannel Sets turn motor controller CAN channel
     * 
     * @param turnEncoderAnalogChannel Sets analog encoder's analog input channel
     * 
     * @param turnZeroAngle Sets wheel angles so they are facing same direction
     */
    public SwerveModule(
            String moduleName,
            int driveMotorCANChannel,
            int turnMotorCANChannel,
            int turnEncoderAnalogChannel,
            double turnZeroAngle) {

        this.moduleName = moduleName;
        
        driveMotor = new CANSparkMax(driveMotorCANChannel, MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(Constants.CHASSIS.DRIVE_MOTOR_CURRENT_LIMIT);
        driveMotor.setIdleMode(IdleMode.kCoast);

        turnMotor = new CANSparkMax(turnMotorCANChannel, MotorType.kBrushless);
        turnMotor.setSmartCurrentLimit(Constants.CHASSIS.TURN_MOTOR_CURRENT_LIMIT);
        turnMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.CHASSIS.DRIVE_POSITION_CONVERSION_FACTOR);
        driveEncoder.setVelocityConversionFactor(Constants.CHASSIS.DRIVE_VELOCITY_CONVERSION_FACTOR);

        turnEncoder = new AnalogInput(turnEncoderAnalogChannel);
        turningPIDController = new PIDController(
            Constants.CHASSIS.TURN_P,
            Constants.CHASSIS.TURN_I,
            Constants.CHASSIS.TURN_D);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        this.turnZeroAngle = turnZeroAngle;
    }

    public void resetDriveEncoderPosition() {
        driveEncoder.setPosition(0.);
    }

    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveRate(), new Rotation2d(-getTurnAngleRadians()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnAngleRadians()));

        final double driveOutput = drivePIDController.calculate(getDriveRate(), state.speedMetersPerSecond);
        final double turnOutput = turningPIDController.calculate(getTurnAngleRadians(), state.angle.getRadians());

        driveMotor.set(driveOutput);
        turnMotor.set(turnOutput);
    }

    public void driveManual(double drive, double turn) {
        driveMotor.set(drive);
        turnMotor.set(turn);
    }

    public void periodic() {
        //SmartDashboard.putNumber(moduleName + " Wheel Speed", getDriveRate());
        SmartDashboard.putNumber(moduleName + " Wheel Angle", getTurnAngleRadians());
        /******* PID Tuning From Shuffleboard Code ********/
        /*
         * double Kp = SmartDashboard.getNumber(m_moduleName + " Turn Kp",
         * m_turnPIDController.getP());
         * SmartDashboard.putNumber("Turn Kp", Kp);
         * double Ki = SmartDashboard.getNumber("Turn Ki", m_turnPIDController.getI());
         * SmartDashboard.putNumber("Turn Ki", Ki);
         * double Kd = SmartDashboard.getNumber("Turn Kd", m_turnPIDController.getD());
         * SmartDashboard.putNumber("Turn Kd", Kd);
         * 
         * m_turnPIDController.setP(Kp);
         * m_turnPIDController.setI(Ki);
         * m_turnPIDController.setD(Kd);
         * 
         * Kp = SmartDashboard.getNumber("Drive Kp", m_drivePIDController.getP());
         * SmartDashboard.putNumber("Drive Kp", Kp);
         * Ki = SmartDashboard.getNumber("Drive Ki", m_drivePIDController.getI());
         * SmartDashboard.putNumber("Drive Ki", Ki);
         * Kd = SmartDashboard.getNumber("Drive Kd", m_drivePIDController.getD());
         * SmartDashboard.putNumber("Drive Kd", Kd);
         * 
         * m_drivePIDController.setP(Kp);
         * m_drivePIDController.setI(Ki);
         * m_drivePIDController.setD(Kd);
         */
    }

    public void testPIDControl(double speed, double angleX, double angleY) {
        double drive = drivePIDController.calculate(getDriveRate(), speed * 5.7);
        driveMotor.set(drive);

        if ((angleX > 0.1) || (angleY > 0.1)) {

            double angle = Math.atan2(angleY, angleX);
            double turn = turningPIDController.calculate(getTurnAngleRadians(), angle);

            turnMotor.set(turn);
        } else {
            turnMotor.set(0.);
        }
    }

    public double getTurningEncoderValue() {
        return turnEncoder.getValue();
    }

    public double getTurnAngleRadians() {
        double value = turnEncoder.getValue();
        value = value / (double) Constants.CHASSIS.TURN_MOTOR_ENCODER_TICKS;
        value = -Math.PI * 2 * (value - 0.5) - turnZeroAngle;
        if (value < -Math.PI) {
            value += 2 * Math.PI;
        }
        return value;
    }

    public double getTurnAngleDegrees() {
        return getTurnAngleRadians() * 180. / Math.PI;
    }

    public double getDriveRate() {
        return driveEncoder.getVelocity();
    }
    public void setBrakeMode(){
        driveMotor.setIdleMode(IdleMode.kBrake);
    }
    public void setCoastMode(){
        driveMotor.setIdleMode(IdleMode.kCoast);
    }
    public void setP(double p){
        drivePIDController.setP(p);
    }
}