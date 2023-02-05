package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class USBCameraSubsystem extends SubsystemBase {
  
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision");

  // Network Table Values
  NetworkTableEntry txNetUSB;
  NetworkTableEntry tyNetUSB;
  NetworkTableEntry taNetUSB;
  NetworkTableEntry tvNetUSB;
  //NetworkTableEntry ledMode = table.getEntry("ledMode");
 
  // read values periodically
  private double tx;
  private double ty;
  private double ta;
  private double tv;

  private GenericEntry shuffleboardUSBCameraX;
  private GenericEntry shuffleboardUSBCameraArea;
  private GenericEntry shuffleboardUSBCameraTargetFound;

  public USBCameraSubsystem() {

    ShuffleboardTab tab = Shuffleboard.getTab("USB Camera");

    txNetUSB = table.getEntry("targetYaw");
    tyNetUSB = table.getEntry("targetPitch");
    taNetUSB = table.getEntry("targetArea");
    tvNetUSB = table.getEntry("hasTarget");
    
    tx = txNetUSB.getDouble(0);
    ty = tyNetUSB.getDouble(0);
    ta = taNetUSB.getDouble(0);
    tv = tvNetUSB.getDouble(0);
        
    shuffleboardUSBCameraX = tab.add("Target X", 0.)
        .withPosition(0, 1)
        .withSize(1, 1)
        .getEntry();
    shuffleboardUSBCameraArea = tab.add("Target Area", 0.)
        .withPosition(2, 1)
        .withSize(1, 1)
        .getEntry();
    shuffleboardUSBCameraTargetFound = tab.add("Target Found", false)
        .withPosition(3, 1)
        .withSize(1, 1)
        .getEntry();
  }

  @Override
  public void periodic() {
    txNetUSB = table.getEntry("tx");
    tyNetUSB = table.getEntry("ty");
    taNetUSB = table.getEntry("ta");
    tvNetUSB = table.getEntry("tv");
    // This method will be called once per scheduler run
    tx = txNetUSB.getDouble(0);
    ty = tyNetUSB.getDouble(0);
    ta = taNetUSB.getDouble(0);
    tv = tvNetUSB.getDouble(0);

    shuffleboardUSBCameraX.setDouble(tx);
    shuffleboardUSBCameraArea.setDouble(ta);
    shuffleboardUSBCameraTargetFound.setBoolean(targetFound());
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
}
