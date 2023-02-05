// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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


    // chassis

  }
  
  public final class CAN {
    //chassis
    public static final int LEFT_DRIVE_MOTOR = 0;
    public static final int RIGHT_DRIVE_MOTOR = 0;

    //arm
    public static final int LEFT_ARM_MOTOR = 0;
    public static final int RIGHT_ARM_MOTOR = 0;

    //intake
    public static final int INTAKE_MOTOR = 0;
  }

  public static final class DIO {
    //intake
    public static final int FLAP_SENSOR = 0;
    public static final int LIMIT_SWITCH = 0;
  }

  public static class ARM {
    //arm
    public static final int FLOOR_POSITION = 0;
    public static final int DRIVE_POSITION = 0;
    public static final int SHELF_POSITION = 0;
    // public static final int LOW_CUBE_POSITION = 0;
    // public static final int LOW_CONE_POSITION = 0;
    // public static final int MEDIUM_CUBE_POSITION = 0;
    // public static final int MEDIUM_CONE_POSITION = 0;
    // public static final int HIGH_CUBE_POSITION = 0;
    // public static final int HIGH_CONE_POSITION = 0;
    public static final int LOW_NODE = 0;
    public static final int MEDIUM_NODE = 0;
    public static final int HIGH_NODE = 0;

    public static final int CUBE_ADJUST = 50;

    public static final int P = 0;
    public static final int I = 0;
    public static final int D = 0;
    public static final int FF = 0;

    // public static final int ARM_SOFT_STOP = 0;

  }

  public static class LED {
    //led
    public static final int PORT = 0;
    public static final int LENGTH = 151;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
