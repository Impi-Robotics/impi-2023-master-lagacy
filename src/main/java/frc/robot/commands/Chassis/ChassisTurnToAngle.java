// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChassisSubsystem;

public class ChassisTurnToAngle extends CommandBase {
  /** Creates a new ChassisTurnToAngle. */
  private final ChassisSubsystem chassisSubsystem;
  private final double desiredAngle;
  private Timer timer;

  public ChassisTurnToAngle(ChassisSubsystem chassisSubsystem, double desiredAngle) {
    this.chassisSubsystem = chassisSubsystem;
    this.desiredAngle = desiredAngle;
    addRequirements(chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassisSubsystem.setSwerveModuleP(0.8);
    chassisSubsystem.setPid(0.1, 0.0, 0.015);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassisSubsystem.turnToAngle(-desiredAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(Math.abs(chassisSubsystem.getAngle()) - Math.abs(desiredAngle)) < 2){
      if (timer == null) {
        timer = new Timer();
        timer.start();
        return false;
      } else if (timer.get() < 0.5) {
        return false;
      } else {
        chassisSubsystem.setPid(0.05, 0., 0.);
        return true;
      }
    } 
    timer = null;
    return false;
  }
}