// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class LimelightSubsystem extends SubsystemBase {

  // Network Tables
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  // Network Table Values
  NetworkTableEntry txNet = table.getEntry("tx");
  NetworkTableEntry tyNet = table.getEntry("ty");
  NetworkTableEntry taNet = table.getEntry("ta");
  NetworkTableEntry tvNet = table.getEntry("tv");
  //NetworkTableEntry ledMode = table.getEntry("ledMode");

  // read values periodically
  private double tx = txNet.getDouble(0);
  private double ty = tyNet.getDouble(0);
  private double ta = taNet.getDouble(0);
  private double tv = tvNet.getDouble(0);

  private GenericEntry shuffleboardLimelightX;
  private GenericEntry shuffleboardLimelightArea;
  private GenericEntry shuffleboardLimelightTargetFound;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    // ShuffleboardTab tab = Shuffleboard.getTab("Limelight");
    //     shuffleboardLimelightX = tab.add("Target X", 0.)
    //             .withPosition(0, 0)
    //             .withSize(1, 1)
    //             .getEntry();
    //     shuffleboardLimelightArea = tab.add("Target Area", 0.)
    //             .withPosition(2, 0)
    //             .withSize(1, 1)
    //             .getEntry();
    //     shuffleboardLimelightTargetFound = tab.add("Target Found", false)
    //             .withPosition(3, 0)
    //             .withSize(1, 1)
    //             .getEntry();
  }

  public boolean targetFound() {
    return (tv == 1.0);
  }

  public double targetFoundDouble() {
    return tv;
  }

  public double targetArea() {
    return ta;
  }

  public double targetXOffset() {
    return tx;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    txNet = table.getEntry("tx");
    tyNet = table.getEntry("ty");
    taNet = table.getEntry("ta");
    tvNet = table.getEntry("tv");

    tx = txNet.getDouble(0);
    ty = tyNet.getDouble(0);
    ta = taNet.getDouble(0);
    tv = tvNet.getDouble(0);

    shuffleboardLimelightX.setDouble(tx);
    shuffleboardLimelightArea.setDouble(ta);
    shuffleboardLimelightTargetFound.setBoolean(targetFound());
  }
}
