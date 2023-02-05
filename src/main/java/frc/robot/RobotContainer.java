package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Arm.*;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

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

	armSubsystem.setDefaultCommand(new Arm_Joystick(armSubsystem, intakeSubsystem, buttonsLeftJoystickY));
    configureBindings();
  }

  private void configureBindings() {
	buttonsA.toggleOnTrue(new Arm_Extend(armSubsystem));
	buttonsA.toggleOnFalse(new Arm_Retract(armSubsystem));
	buttonsB.toggleOnTrue(new Grabber_Close(armSubsystem));
	buttonsB.toggleOnFalse(new Grabber_Open(armSubsystem));
	buttonsX.toggleOnTrue(new Grabber_Up(armSubsystem));
	buttonsX.toggleOnFalse(new Grabber_Down(armSubsystem));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
