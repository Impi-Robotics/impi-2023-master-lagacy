package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Swerve_TurnToAngle extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final double desiredAngle;
  private Timer timer;
  
  public Swerve_TurnToAngle(SwerveSubsystem swerveSubsystem, double desiredAngle) {
    this.swerveSubsystem = swerveSubsystem;
    this.desiredAngle = desiredAngle;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    swerveSubsystem.turnToAngle(desiredAngle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(Math.abs(Math.abs(swerveSubsystem.getGyroAngle()) - Math.abs(desiredAngle)) < 2){
      if (timer == null) {
        timer = new Timer();
        timer.start();
        return false;
      } else if (timer.get() < 0.5) {
        return false;
      } else {
        return true;
      }
    } 
    timer = null;
    return false;
  }
}
