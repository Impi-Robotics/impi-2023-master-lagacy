// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ChassisSubsystem extends SubsystemBase {
  private VictorSP leftDriveMotor;
  private VictorSP rightDriveMotor;
  private DifferentialDrive drive;
  /** Creates a new ChassisSubsystem. */
  public ChassisSubsystem() {
    leftDriveMotor = new VictorSP(Constants.CAN.LEFT_DRIVE_MOTOR);
    rightDriveMotor = new VictorSP(Constants.CAN.RIGHT_DRIVE_MOTOR);
    drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void arcadeDrive(double ySpeed, double rotation){
    drive.arcadeDrive(-ySpeed, -rotation);
  }

  public Command exampleMethodCommand() {
    return null;
  }
}