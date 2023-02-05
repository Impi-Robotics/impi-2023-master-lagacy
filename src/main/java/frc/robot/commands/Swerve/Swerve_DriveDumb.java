package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ImpiLib2023;
import frc.robot.subsystems.SwerveSubsystem;

public class Swerve_DriveDumb extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private DoubleSupplier xSpeed;
  private DoubleSupplier ySpeed;
  private DoubleSupplier rotR;
  private DoubleSupplier rotL;
  private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

  public Swerve_DriveDumb(SwerveSubsystem swerveSubsystem, 
                                    DoubleSupplier xSpeed,
                                    DoubleSupplier ySpeed,
                                    DoubleSupplier rotR,
                                    DoubleSupplier rotL) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotR = rotR;
    this.rotL = rotL;
    xLimiter = new SlewRateLimiter(40);
    yLimiter = new SlewRateLimiter(40);
    //25 if tippy

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xValue = ImpiLib2023.parseJoystick(xSpeed) * Constants.CHASSIS.MAX_METERS_PER_SECOND;
    double yValue = ImpiLib2023.parseJoystick(ySpeed) * Constants.CHASSIS.MAX_METERS_PER_SECOND;
    double rotValue = (ImpiLib2023.parseJoystick(rotR) - ImpiLib2023.parseJoystick(rotL)) * 10;
    xValue = xLimiter.calculate(xValue);
    yValue = yLimiter.calculate(yValue);
    swerveSubsystem.dumbDrive(xValue, -yValue, -rotValue);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
