// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Arm_RunToLowNode extends CommandBase {
  /** Creates a new Arm_RunToHighNode. */
  private ArmSubsystem armSubsystem;
  private IntakeSubsystem intakeSubsystem;
  public Arm_RunToLowNode(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(armSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.ArmExtend();
    if((armSubsystem.getArmState() == armSubsystem.armStates[2]) || ((armSubsystem.getArmState() == armSubsystem.armStates[6]) && armSubsystem.getMinPassThrough())) {
      intakeSubsystem.FlapOpen();      
    } else {
      intakeSubsystem.FlapClose();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.GoToLowNode();
    if(armSubsystem.getPassThrough()) {
      intakeSubsystem.FlapOpen();
    } else {
      intakeSubsystem.FlapClose();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.ArmStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}