package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.LED.*;
import frc.robot.commands.Swerve.Swerve_DriveDumb;
import frc.robot.commands.Swerve.Swerve_DriveStraight;
import frc.robot.commands.Swerve.Swerve_ResetGyro;
import frc.robot.commands.Swerve.Swerve_StraightenWheels;
import frc.robot.commands.Swerve.Swerve_TurnToAngle;
import frc.robot.commands.Swerve.Swerve_DriveField;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	// Subsystems
	private final ArmSubsystem armSubsystem = new ArmSubsystem();
	// Xbox Controller Stuff
	private final XboxController driverController = new XboxController(Constants.OI.OI_DRIVER_CONTROLLER);
	private final XboxController buttonsController = new XboxController(Constants.OI.OI_BUTTONS_CONTROLLER);

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

	armSubsystem.setDefaultCommand();

    configureBindings();
  }

  private void configureBindings() {
    buttonsA.onTrue(new CubeModeNotHolding(ledSubsystem));
	buttonsB.onTrue(new CubeModeHolding(ledSubsystem));
    buttonsX.onTrue(new ConeModeNotHolding(ledSubsystem));
	buttonsY.onTrue(new ConeModeHolding(ledSubsystem));
	buttonsLeftBumper.onTrue(new Aligned(ledSubsystem));
	buttonsRightBumper.onTrue(new NotAligned(ledSubsystem));
	driverLBumper.toggleOnTrue(new Swerve_DriveField(swerveSubsystem, driverLeftJoystickX, driverLeftJoystickY, driverRightTrigger, driverLeftTrigger, true));
    driverA.onTrue(new Swerve_ResetGyro(swerveSubsystem, 90.0));
	//3)
    //buttonsA.onTrue(new Swerve_TurnToAngle(swerveSubsystem, 90));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
