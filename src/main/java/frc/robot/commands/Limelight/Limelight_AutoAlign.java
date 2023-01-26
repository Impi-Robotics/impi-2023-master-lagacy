// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.lang.Math;
import java.util.function.DoubleSupplier;

public class Limelight_AutoAlign extends CommandBase {

  private LimelightSubsystem limelightSubsystem;
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
  private double turnP = 0.05;

  //Desired speeds for wheels
  private double rotate = 0;
  private double drive = 0;
  
  /** Creates a new Limelight_AutoDrive. */
  public Limelight_AutoAlign(LimelightSubsystem limelightSubsystem, ChassisSubsystem chassisSubsystem, boolean auto) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelightSubsystem = limelightSubsystem;
    this.chassisSubsystem = chassisSubsystem;
    this.auto = auto;
    addRequirements(chassisSubsystem);
  }

  public void updateLimelight() {
    tv = limelightSubsystem.targetFoundDouble(); 
    ta = limelightSubsystem.targetArea();
    tx = limelightSubsystem.targetXOffset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateLimelight();
    // if (Math.abs(tx) < 5.0) {
    //   turnP = 0.3;
    // } 
    rotate = tx * turnP;
    drive = (desiredArea - ta) * driveP;
    // updateLimelight();
    // if (tv == 1.0) {
    //   while (Math.abs(tx) > 0.5) {
    //     updateLimelight();
    //     if (tv == 1.0) break;
    //     chassisSubsystem.arcadeDrive(0.0, (autoRotation * (Math.signum(tx) * -1.0)));
    //   }
    //   while (ta < 9.0) {
    //     updateLimelight();
    //     if (tv == 1.0) break;
    //     chassisSubsystem.arcadeDrive(autoSpeed, 0.0);
    //   }
    // }
    if(drive > 0.6){
      drive = 0.6;
    }
    chassisSubsystem.arcadeDrive(drive, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (limelightSubsystem.targetArea() > 2.0) {
      return true;
    }
    return false;
  }
}
