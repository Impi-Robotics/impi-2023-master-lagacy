package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Chassis.Chassis_ArcadeDrive;
import frc.robot.commands.Intake.Intake_Joystick;
import frc.robot.commands.LED.*;
import frc.robot.commands.Limelight.*;
import frc.robot.commands.USBCamera.USBCamera_AutoAlign;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final USBCameraSubsystem usbCameraSubsystem = new USBCameraSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private final Chassis_ArcadeDrive autoCommand;
  // Xbox Controller Stuff
	private final XboxController driverController = new XboxController(Constants.IO.Driver);
	private final XboxController buttonsController = new XboxController(Constants.IO.Buttons);

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
    intakeSubsystem.setDefaultCommand(new Intake_Joystick(intakeSubsystem, driverLeftTrigger, driverRightTrigger));
    chassisSubsystem.setDefaultCommand(new Chassis_ArcadeDrive(chassisSubsystem, driverLeftJoystickY, driverRightJoystickX));
    autoCommand = new Chassis_ArcadeDrive(chassisSubsystem, driverLeftJoystickY, driverRightJoystickX);
    swerveSubsystem.setDefaultCommand(new Swerve_DriveDumb(swerveSubsystem,
                                                      driverLeftJoystickY,
                                                      driverLeftJoystickX,
                                                      driverRightTrigger,
                                                      driverLeftTrigger));

    configureBindings();
  }

  private void configureBindings() {
    buttonsA.onTrue(new Swerve_StraightenWheels(swerveSubsystem));
    //buttonsA.onTrue(new Swerve_TurnToAngle(swerveSubsystem, 90));
    driverRBumper.whileTrue(new Limelight_AutoAlign(limelightSubsystem, chassisSubsystem, true));
    driverRBumper.whileFalse(new Limelight_AutoAlign(limelightSubsystem, chassisSubsystem, false));
    driverA.whileTrue(new USBCamera_AutoAlign(usbCameraSubsystem, chassisSubsystem));
  	driverLBumper.toggleOnTrue(new PipelineSwitch(limelightSubsystem));
	  buttonsA.onTrue(new ConeMode(ledSubsystem));
  	buttonsB.onTrue(new CubeMode(ledSubsystem));
	  buttonsX.onTrue(new CubeLoading(ledSubsystem));
  	buttonsY.onTrue(new CubeLoaded(ledSubsystem));
	  buttonsLeftBumper.onTrue(new ObjectHeld(ledSubsystem));
  	buttonsRightBumper.onTrue(new ObjectVacant(ledSubsystem));
  	buttonsStart.onTrue(new TargetSeen(ledSubsystem));
	  buttonsSelect.onTrue(new TargetAligned(ledSubsystem));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
