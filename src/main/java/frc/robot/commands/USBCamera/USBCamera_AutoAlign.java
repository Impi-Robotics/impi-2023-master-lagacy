// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.USBCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.USBCameraSubsystem;

public class USBCamera_AutoAlign extends CommandBase {

  private USBCameraSubsystem usbCameraSubsystem;
  private ChassisSubsystem chassisSubsystem;
  private double autoSpeed = 0.7;//How fast is drives when its not within a certain distance
  private double autoRotation = 0.7;//How fast it rotates when its not within a certain range(x offset)
  private double tv; //Target found
  private double ta; //Target Area
  private double desiredArea = 4.0;
  private double tx; //X Offset
  private boolean auto = false;

  //Proportions
  private double driveP = 0.25;
  private double turnP = 5;

  //Desired speeds for wheels
  private double rotate = 0;
  private double drive = 0;
  
  public USBCamera_AutoAlign(USBCameraSubsystem usbCameraSubsystem, ChassisSubsystem chassisSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.usbCameraSubsystem = usbCameraSubsystem;
    this.chassisSubsystem = chassisSubsystem;
    addRequirements(chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tv = usbCameraSubsystem.targetFoundDouble(); 
    ta = usbCameraSubsystem.targetArea();
    tx = usbCameraSubsystem.targetXOffset();

    rotate = tx * turnP;
    drive = (desiredArea - ta) * driveP;
    if(drive > 0.6){
      drive = 0.6;
    }
    if(ta > 0.8){
      chassisSubsystem.arcadeDrive(0, 0);
    }
    else{
      chassisSubsystem.arcadeDrive(drive, rotate);
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (usbCameraSubsystem.targetArea() > 0.8) {
      return true;
    }
    return false;
  }
}
