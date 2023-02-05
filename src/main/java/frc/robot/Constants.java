package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public final class OI {
    // Driver Controllers
    public static final int DRIVER_CONTROLLER = 0;
    public static final int BUTTONS_CONTROLLER = 1;
  }

  public final class PCM {
    // arm
    public static final int LEFT_ARM_PISTON = 0;
    public static final int RIGHT_ARM_PISTON = 0;

    // grabber
    public static final int LEFT_GRABBER_PISTON = 0;
    public static final int RIGHT_GRABBER_PISTON = 0;
    public static final int FLIP_GRABBER_PISTON = 0;

    // intake
    public static final int LEFT_INTAKE_PISTON = 0;
    public static final int RIGHT_INTAKE_PISTON = 0;
    public static final int LEFT_FLAP_PISTON = 0;
    public static final int RIGHT_FLAP_PISTON = 0;

  }
  public static final class CAN{
    public static final int CHASSIS_FRONT_LEFT_DRIVE_MOTOR = 10;
    public static final int CHASSIS_FRONT_LEFT_TURN_MOTOR = 11;
    public static final int CHASSIS_FRONT_RIGHT_DRIVE_MOTOR = 12;
    public static final int CHASSIS_FRONT_RIGHT_TURN_MOTOR = 13;
    public static final int CHASSIS_BACK_LEFT_DRIVE_MOTOR = 14;
    public static final int CHASSIS_BACK_LEFT_TURN_MOTOR = 15;
    public static final int CHASSIS_BACK_RIGHT_DRIVE_MOTOR = 16;
    public static final int CHASSIS_BACK_RIGHT_TURN_MOTOR = 17;

    //arm
    public static final int ARM_LEFT_MOTOR = 0;
    public static final int ARM_RIGHT_MOTOR = 0;

    //intake
    public static final int INTAKE_MOTOR = 0;
  }

  public static final class DIO {
    public static final int FLAP_SENSOR = 0;
    public static final int FLIP_SENSOR = 0;
    public static final int LIMIT_SWITCH = 0;
  }

  public final class AI {
    // Analog Inputs
    public static final int CHASSIS_FRONT_LEFT_DRIVE_ENCODER = 0;
    public static final int CHASSIS_FRONT_RIGHT_DRIVE_ENCODER = 1;
    public static final int CHASSIS_BACK_LEFT_DRIVE_ENCODER = 2;
    public static final int CHASSIS_BACK_RIGHT_DRIVE_ENCODER = 3;
  }
  
  public static class ARM {
    public static final int FLOOR_POSITION = 0;
    public static final int DRIVE_POSITION = 0;
    public static final int SHELF_POSITION = 0;
    public static final int LOW_NODE = 0;
    public static final int MEDIUM_NODE = 0;
    public static final int HIGH_NODE = 0;

    public static final int CUBE_ADJUST = 50;

    public static final int P = 0;
    public static final int I = 0;
    public static final int D = 0;
    public static final int FF = 0;

    // public static final int ARM_SOFT_STOP = 0;

    public static final int ARM_SOFT_STOP = 0;
  }

  public static final class CHASSIS{
    //Limits
    public static final double DRIVE_SMART_CURRENT_LIMIT = 50;

    //public static final double MAX_METERS_PER_SECOND = Units.feetToMeters(14.5);
    //public static final double MAX_METERS_PER_SECOND_SQUARED = Math.pow(MAX_METERS_PER_SECOND, 2);
    public static final double MAX_METERS_PER_SECOND = 5;
    public static final double MAX_METERS_PER_SECOND_SQUARED = 500;
    // public static final double MAX_METERS_PER_SECOND = 20000;

    //Will calculate today
    public static final double SWERVE_FRONT_RIGHT_ZERO_ANGLE = 0.84 + Units.degreesToRadians(5);
    public static final double SWERVE_FRONT_LEFT_ZERO_ANGLE = -3.015 + Units.degreesToRadians(90);
    public static final double SWERVE_BACK_RIGHT_ZERO_ANGLE = 2.62 + Units.degreesToRadians(8);
    public static final double SWERVE_BACK_LEFT_ZERO_ANGLE = -1.80 + Units.degreesToRadians(5);
    
    public static final double TRACK_WIDTH = Units.inchesToMeters(19);
    public static final double WHEEL_BASE = Units.inchesToMeters(19);
    // public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
    //   new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    //   new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
    //   new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    //   new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2));
    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2));


  }
  
  public static class SWERVE{
    public static double TURN_MOTOR_ENCODER_TICKS = 4096;
    //Drive Pid:
    public static double DRIVE_P = 0.25;
    public static double DRIVE_I = 0.01;
    public static double DRIVE_D = 0;
    //Turn Pid:
    public static double TURN_P = 0.2;
    public static double TURN_I = 0.;
    public static double TURN_D = 0.;
    //Limits
    public static final double DRIVE_SMART_CURRENT_LIMIT = 30;

    //Calculations
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.);
    //gear ratio = 6.75... 1/6.75 = 0.1482
    //public static final double DRIVE_GEAR_RATIO = 0.1482;
    public static final double DRIVE_GEAR_RATIO = 0.1016;
    public static final double DRIVE_POSITION_CONVERSION_FACTOR = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_POSITION_CONVERSION_FACTOR / 60;
    //gear ratio = 0.078... 1/12.8
    public static final double TURN_GEAR_RATIO = 0.0781;
    public static final double TURN_ENCODER_ROT_TO_RADIANS = TURN_GEAR_RATIO * 2 * Math.PI;
    public static final double TURN_RPM_TO_RAD_PER_SECOND = TURN_ENCODER_ROT_TO_RADIANS / 60;
  }

  public static class LED {
    //led
    public static final int PORT = 0;
    public static final int LENGTH = 0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}