package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Chassis.Chassis_AlignmentLimelight;
import frc.robot.commands.Chassis.Chassis_ArcadeDrive;
import frc.robot.commands.Limelight.*;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.USBCameraSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Swerve_TurnToAngle;
import frc.robot.commands.Swerve.Swerve_DriveDumb;
import frc.robot.commands.Swerve.Swerve_DriveStraight;
import frc.robot.commands.Swerve.Swerve_StraightenWheels;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //Subsystems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final USBCameraSubsystem usbCameraSubsystem = new USBCameraSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

//   private final Chassis_ArcadeDrive autoCommand;
  // Xbox Controller Stuff
	private final XboxController driverController = new XboxController(Constants.OI.DRIVER_CONTROLLER);
	private final XboxController buttonsController = new XboxController(Constants.OI.BUTTONS_CONTROLLER);

	private final JoystickButton driverA = new JoystickButton(driverController, XboxController.Button.kA.value);
	private final JoystickButton driverB = new JoystickButton(driverController, XboxController.Button.kB.value);
	private final JoystickButton driverX = new JoystickButton(driverController, XboxController.Button.kX.value);
	private final JoystickButton driverY = new JoystickButton(driverController, XboxController.Button.kY.value);
	private final JoystickButton driverRBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
	private final JoystickButton driverLBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
	private final JoystickButton driverSelect = new JoystickButton(driverController, XboxController.Button.kBack.value);
	private final JoystickButton driverStart = new JoystickButton(driverController, XboxController.Button.kStart.value);

	private final DoubleSupplier driverLeftJoystickX = () -> driverController.getLeftX();
	private final DoubleSupplier driverLeftJoystickY = () -> driverController.getLeftY();
	private final DoubleSupplier driverLeftTrigger = () -> driverController.getLeftTriggerAxis();
	private final DoubleSupplier driverRightJoystickX = () -> driverController.getRightX();
	private final DoubleSupplier driverRightJoystickY = () -> driverController.getRightY();
	private final DoubleSupplier driverRightTrigger = () -> driverController.getRightTriggerAxis();
	private final IntSupplier driverDpad = () -> driverController.getPOV();

	private final JoystickButton buttonsA = new JoystickButton(buttonsController, XboxController.Button.kA.value);
	private final JoystickButton buttonsB = new JoystickButton(buttonsController, XboxController.Button.kB.value);
	private final JoystickButton buttonsX = new JoystickButton(buttonsController, XboxController.Button.kX.value);
	private final JoystickButton buttonsY = new JoystickButton(buttonsController, XboxController.Button.kY.value);
	private final JoystickButton buttonsLeftBumper = new JoystickButton(buttonsController, XboxController.Button.kLeftBumper.value);
	private final JoystickButton buttonsRightBumper = new JoystickButton(buttonsController, XboxController.Button.kRightBumper.value);
	private final JoystickButton buttonsSelect = new JoystickButton(buttonsController, XboxController.Button.kBack.value);
	private final JoystickButton buttonsStart = new JoystickButton(buttonsController, XboxController.Button.kStart.value);

	private final DoubleSupplier buttonsLeftJoystickX = () -> buttonsController.getLeftX();
	private final DoubleSupplier buttonsLeftJoystickY = () -> buttonsController.getLeftY();
	private final DoubleSupplier buttonsLeftTrigger = () -> buttonsController.getLeftTriggerAxis();
	private final DoubleSupplier buttonsRightJoystickX = () -> buttonsController.getRightX();
	private final DoubleSupplier buttonsRightJoystickY = () -> buttonsController.getRightY();
	private final DoubleSupplier buttonsRightTrigger = () -> buttonsController.getRightTriggerAxis();
	private final IntSupplier buttonsDpad = () -> buttonsController.getPOV();

  public RobotContainer() {
    // autoCommand = new Chassis_ArcadeDrive(swerveSubsystem, driverLeftJoystickY, driverRightJoystickX);
    swerveSubsystem.setDefaultCommand(new Swerve_DriveDumb(swerveSubsystem,
                                                      driverLeftJoystickY,
                                                      driverLeftJoystickX,
                                                      driverRightTrigger,
                                                      driverLeftTrigger));
	
    configureBindings();
  }

  private void configureBindings() {
    driverA.onTrue(new Swerve_StraightenWheels(swerveSubsystem));
    //buttonsA.onTrue(new Swerve_TurnToAngle(swerveSubsystem, 90));
	buttonsA.onTrue(new PipelineSwitch(limelightSubsystem));
	buttonsB.onTrue(new Chassis_AlignmentLimelight());
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
