// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import org.apache.commons.lang3.function.ToBooleanBiFunction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class Swerve_AutoAim extends CommandBase {
  /** Creates a new Swerve_AutoAim. */
  private ChassisSubsystem chassisSubsystem;
  private LimelightSubsystem limelightSubsystem;

  public Swerve_AutoAim(ChassisSubsystem chassisSubsystem, LimelightSubsystem limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassisSubsystem = chassisSubsystem;
    this.chassisSubsystem = chassisSubsystem;    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //while(angleOffset > 2){
      turnToAngle(0) -> 
      strafe() -> stop when tx is around 1
      driveToTarget() -> stop when ta is certain val
    }
    chassisSubsystem.setDesiredAngle(limelightSubsystem.targetXOffset());
    chassisSubsystem.drive(0., 0., 0., true, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {/*boolean start toString(on)ToBooleanBiFunction iFunction on swerve;*/
    chassisSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (chassisSubsystem.isTargetCentered()) {
        return true;
    }
    return false;

  }
}