package frc.robot.commands.Chassis;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ImpiLib2023;
import frc.robot.subsystems.ChassisSubsystem;

public class Chassis_ArcadeDrive extends CommandBase {

  private ChassisSubsystem chassisSubsystem;
  private DoubleSupplier ySpeed;
  private DoubleSupplier rotation;

  /** Creates a new Chassis_ArcadeDrive. */
  public Chassis_ArcadeDrive(ChassisSubsystem chassisSubsystem, DoubleSupplier ySpeed, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassisSubsystem = chassisSubsystem;
    this.ySpeed = ySpeed;
    this.rotation = rotation;
    addRequirements(chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassisSubsystem.arcadeDrive(ImpiLib2023.deadzone(-ySpeed.getAsDouble(), 0.1), ImpiLib2023.deadzone(-rotation.getAsDouble(), 0.1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}