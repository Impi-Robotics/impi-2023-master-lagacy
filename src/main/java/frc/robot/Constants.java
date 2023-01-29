package frc.robot;

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
    public static final int LEFT_DRIVE_MOTOR = 0;
    public static final int RIGHT_DRIVE_MOTOR = 0;

    //arm
    public static final int LEFT_ARM_MOTOR = 0;
    public static final int RIGHT_ARM_MOTOR = 0;

    //intake
    public static final int INTAKE_MOTOR = 0;
  }

  public static final class DIO {
    public static final int FLAP_SENSOR = 0;
    public static final int FLIP_SENSOR = 0;
    public static final int LIMIT_SWITCH = 0;
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

  public static class LED {
    public static final int PORT = 0;
    public static final int LENGTH = 0;
  }

  public static class IO {
    public static final int Driver = 0;
    public static final int Buttons = 1;
  }
}
