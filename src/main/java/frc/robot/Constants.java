package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public final class PCM {
    // arm
    public static final int ARM_PISTON = 0;

    // grabber
    public static final int GRABBER_PISTON = 0;
    public static final int FLIP_GRABBER_PISTON = 0;

    // intake
    public static final int INTAKE_PISTON = 0;
    public static final int FLAP_PISTON = 0;
  }
  
  public final class CAN {
    //chassis
    public static final int CHASSIS_FRONT_LEFT_DRIVE_MOTOR = 17;
    public static final int CHASSIS_FRONT_LEFT_TURN_MOTOR = 18;
    public static final int CHASSIS_FRONT_RIGHT_DRIVE_MOTOR = 11;
    public static final int CHASSIS_FRONT_RIGHT_TURN_MOTOR = 12;
    public static final int CHASSIS_REAR_LEFT_DRIVE_MOTOR = 16;
    public static final int CHASSIS_REAR_LEFT_TURN_MOTOR = 15;
    public static final int CHASSIS_REAR_RIGHT_DRIVE_MOTOR = 13;
    public static final int CHASSIS_REAR_RIGHT_TURN_MOTOR = 14;

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
    public static final int FRONT_LEFT_DRIVE_ENCODER = 0;
    public static final int FRONT_RIGHT_DRIVE_ENCODER = 1;
    public static final int REAR_LEFT_DRIVE_ENCODER = 2;
    public static final int REAR_RIGHT_DRIVE_ENCODER = 3;
  }
  
  public static class ARM {
    public static final int FLOOR_POSITION = 0;
    public static final int DRIVE_POSITION = 0;
    public static final int SHELF_POSITION = 0;
    public static final int LOW_NODE = 0;
    public static final int MEDIUM_NODE = 0;
    public static final int HIGH_NODE = 0;

    // Encoder adjustment for cube mode
    public static final int CUBE_ADJUST = 50;
    
    public static final int P = 0;
    public static final int I = 0;
    public static final int D = 0;
    public static final int FF = 0;

    // Point nearest to Conveyor/Drive pos when flap should be opened or closed
    public static final int MIN_PASS_THROUGH = 0;
    // Point farthest from Conveyor/Drive pos when flap should be opened or closed
    public static final int MAX_PASS_THROUGH = 0;

    public static final int ARM_SOFT_STOP = 0;
  }

  public final class CHASSIS {
    //AUTO
    public static final double PATH_DRIVE_P = 1.;
    public static final double PATH_DRIVE_I = 0.;
    public static final double PATH_DRIVE_D = 0.; 

    public static final double PATH_THETA_P = 4.;
    public static final double PATH_THETA_I = 0.;
    public static final double PATH_THETA_D = 0.;
    // Drive Motors
    //0.8
    public static final double DRIVE_P = 1.5;
    public static final double DRIVE_I = 0.;
    public static final double DRIVE_D = 0.;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 30;

    // Turn Motors
    public static final double TURN_P = 0.2;
    public static final double TURN_I = 0.;
    public static final double TURN_D = 0.;

    public static final int TURN_MOTOR_CURRENT_LIMIT = 30;

    public static final int TURN_MOTOR_ENCODER_TICKS = 4096;

    // Calculations
    public static final double MAX_METERS_PER_SECOND = 7;
    public static final double MAX_METERS_PER_SECOND_SQUARED = 500; // 100
    public static final double MAX_TURN_ANGULAR_SPEED = 500. * Math.PI;
    public static final double MAX_TURN_ANGULAR_ACCELERATION = 500. * Math.PI;

    public static final double WHEEL_DIAMETER_METERS = 0.1016;
    public static final double DRIVE_GEAR_RATIO = 0.1529;
    public static final double DRIVE_POSITION_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER_METERS * DRIVE_GEAR_RATIO;
    public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = 
            DRIVE_GEAR_RATIO * WHEEL_DIAMETER_METERS * Math.PI / 60.;
  }

  public static class Swerve{
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
}
