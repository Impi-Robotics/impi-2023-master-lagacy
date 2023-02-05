package frc.robot.commands.Chassis;

import frc.robot.Constants;
import frc.robot.ImpiLib2023;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ChassisCommand extends CommandBase {
    
    private final ChassisSubsystem m_subsystem;
    private final LimelightSubsystem limelightSubsystem;

    private DoubleSupplier turnTriggerLeft;
    private DoubleSupplier turnTriggerRight;
    private DoubleSupplier driveJoystickX;
    private DoubleSupplier driveJoystickY;
    private DoubleSupplier rotateJoystickX;
    private DoubleSupplier rotateJoystickY;
    private JoystickButton driverRBumper;
    private JoystickButton driverLBumper;
    boolean trackTarget;

    public ChassisCommand(ChassisSubsystem subsystem, LimelightSubsystem limelightSubsystem,
            DoubleSupplier turnTriggerLeft, DoubleSupplier turnTriggerRight,
            DoubleSupplier driveJoystickX, DoubleSupplier driveJoystickY, DoubleSupplier rotateJoystickX,
            DoubleSupplier rotateJoystickY, JoystickButton driverRBumper, JoystickButton driverLBumper) {
        m_subsystem = subsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.turnTriggerLeft = turnTriggerLeft;
        this.turnTriggerRight = turnTriggerRight;
        this.driveJoystickX = driveJoystickX;
        this.driveJoystickY = driveJoystickY;
        this.rotateJoystickX = rotateJoystickX;
        this.rotateJoystickY = rotateJoystickY;
        this.driverRBumper = driverRBumper;
        this.driverLBumper = driverLBumper;

        addRequirements(subsystem, limelightSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_subsystem.updateGyro();
        
        double rotation = 0;
        double xValue = -ImpiLib2023.parseJoystick(driveJoystickX, 0.15) * Constants.CHASSIS.MAX_METERS_PER_SECOND;
        double yValue = ImpiLib2023.parseJoystick(driveJoystickY, 0.15) * Constants.CHASSIS.MAX_METERS_PER_SECOND;
        boolean stickUsed = Math.hypot(rotateJoystickX.getAsDouble(), rotateJoystickY.getAsDouble()) > 0.5;
        if (stickUsed) {
            //m_subsystem.turnToTargetOdometryOff();
            m_subsystem.setDesiredAngle(180. / Math.PI * Math.atan2(-ImpiLib2023.parseJoystick(rotateJoystickX, 0.1),
                    -ImpiLib2023.parseJoystick(rotateJoystickY, 0.1)));
        } else {
            /*
            if (Math.abs(turnTriggerLeft.getAsDouble()) > 0.5 || Math.abs(turnTriggerRight.getAsDouble()) > 0.5) {
                m_subsystem.turnToTargetOdometryOff();

            }
            */
            rotation = (ImpiLib2023.parseJoystick(turnTriggerLeft, 0.05)
                    - ImpiLib2023.parseJoystick(turnTriggerRight, 0.05)) * 5;
        }
        // Vision
        if (driverRBumper.getAsBoolean()) {
            double xOffset = limelightSubsystem.targetXOffset();
            if (Math.abs(xOffset) > 2.) {
                m_subsystem.setDesiredAngle(xOffset);
            }
        }
        boolean fieldOriented = !driverLBumper.getAsBoolean();
        m_subsystem.drive(-xValue, -yValue, rotation, fieldOriented, driverRBumper.getAsBoolean(), stickUsed);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
