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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    //Motor Controllers
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    //Encoder
    private RelativeEncoder driveEncoder;
    //Absolute Encoder
    private AnalogInput turnAbsoluteEncoder;

    //PID
    //NEED TO FIND PID VALUES
    // private final ProfiledPIDController drivePidController = new ProfiledPIDController(
    //     Constants.Swerve.DRIVE_P,
    //     Constants.Swerve.DRIVE_I,
    //     Constants.Swerve.DRIVE_D,
    //       //NEED TO FIND MAX VELO AND MAX ACCEL
    //         new TrapezoidProfile.Constraints(
    //             Constants.CHASSIS.MAX_METERS_PER_SECOND,
    //             Constants.CHASSIS.MAX_METERS_PER_SECOND_SQUARED));
    private final PIDController drivePidController = new PIDController(
        Constants.SWERVE.DRIVE_P,Constants.SWERVE.DRIVE_I, Constants.SWERVE.DRIVE_D);
    private final PIDController turnPidController;
    private double zeroAngle;
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
        int driveMotorCANID,
        int turnMotorCANID,
        int turnEncoderAnalogPort,
        double turnZeroAngle,
        boolean isInverted){

        this.moduleName = moduleName;
        driveMotor = new CANSparkMax(driveMotorCANID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorCANID, MotorType.kBrushless);
        setIdleModes();
        setSmartLimit();
        driveMotor.setInverted(isInverted);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.SWERVE.DRIVE_POSITION_CONVERSION_FACTOR);
        driveEncoder.setVelocityConversionFactor(Constants.SWERVE.DRIVE_RPM_TO_METERS_PER_SECOND);

        turnAbsoluteEncoder = new AnalogInput(turnEncoderAnalogPort);
        turnPidController = new PIDController(
            Constants.SWERVE.TURN_P, 
            Constants.SWERVE.TURN_I,
            Constants.SWERVE.TURN_D);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);
        this.zeroAngle = turnZeroAngle;
        //Might need for conversion factors to actually work...
        resetEncoders();
        burnFlash();
    }
    public void setIdleModes(){
        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kCoast);
    }
    public void setSmartLimit(){
        driveMotor.setSmartCurrentLimit(30);
        turnMotor.setSmartCurrentLimit(30);
    }
    public void burnFlash(){
        driveMotor.burnFlash();
        turnMotor.burnFlash();
    }
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    public double getTurnEncoderValue(){
        return turnAbsoluteEncoder.getValue();
    }
    public double getTurnAngleRadians(){
        double angle = getTurnEncoderValue();
        angle = angle / Constants.SWERVE.TURN_MOTOR_ENCODER_TICKS;
        angle = -Math.PI * 2 * (angle - 0.5) - zeroAngle;
         if(angle < -Math.PI){
             angle += 2 * Math.PI;
        }
        return angle;
    }
    public void resetEncoders(){
        driveEncoder.setPosition(0);
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnAngleRadians()));
        // return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnAngleRadians()));
    }
    public void setDesiredState(SwerveModuleState desiredState){
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnAngleRadians()));
        final double driveOutput = drivePidController.calculate(getDriveVelocity(), desiredState.speedMetersPerSecond);
        final double turnOutput = turnPidController.calculate(getTurnAngleRadians(), desiredState.angle.getRadians());
        driveMotor.set(driveOutput);
        turnMotor.set(turnOutput);
    }
    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    //TESTING
    public void driveManual(double drive, double turn){
        driveMotor.set(drive);
        driveMotor.set(turn);
    }

    public void periodic(){
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

}