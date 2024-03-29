package frc.robot;

import javax.script.Bindings;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.Utils;

/**
 * This class cointains definitions for the robot's hardware devices.
 * These include components such as motor controllers and soleniods used by the subsystems.
 */
public class OI {
    private static final double JOYSTICK_DEADZONE_INNER = 0.05; // Below the inner value the input is zero
    private static final double JOYSTICK_DEADZONE_OUTER = 0.2; // Between the inner and outer value the input is interpolated towards it's actual value

    static double newXInput = 0.0;
    static double newYInput = 0.0;
    static double prevXInput = 0.0;
    static double prevYInput = 0.0;

    public static double getXDriveInput(){
        // grab old input from controller
        prevXInput = newXInput;
        // read new input from controller
        newXInput = OI.driverController.getLeftX();
        // implement deadzoning
        newXInput = applyDeadzone(newXInput);

        // apply speed modifier
        newXInput *= getSpeedMultiplier();

        // read max accel from shuffleboard
        double maxAccel = RobotContainer.drivetrain.maxAccel.getDouble(0.02);
        // limit the acceleration
        newXInput = (newXInput - prevXInput) > maxAccel ? prevXInput + maxAccel : newXInput;
        newXInput = (newXInput - prevXInput) < -1 * maxAccel ? prevXInput - maxAccel : newXInput;
        return (newXInput);
    }

    public static double getYDriveInput(){
        // grab old input from controller
        prevYInput = newYInput;
        // read new input from controller
        newYInput = OI.driverController.getLeftY();
        // implement deadzoning
        newYInput = applyDeadzone(newYInput);

        // apply speed modifier
        newYInput *= getSpeedMultiplier();

        // read max accel from shuffleboard
        double maxAccel = RobotContainer.drivetrain.maxAccel.getDouble(0.02);
        // limit the acceleration
        newYInput = (newYInput - prevYInput) > maxAccel ? prevYInput + maxAccel : newYInput;
        newYInput = (newYInput - prevYInput) < -1 * maxAccel ? prevYInput - maxAccel : newYInput;
        return (newYInput);
    }

    // Smooths deadzone over range
    public static double applyDeadzone(double input){
        return Math.abs(input) > JOYSTICK_DEADZONE_OUTER ? 
        input : 
        (Math.abs(input) < JOYSTICK_DEADZONE_INNER ? 0 : (Utils.Lerp(0, input, Utils.InvLerp(JOYSTICK_DEADZONE_INNER, JOYSTICK_DEADZONE_OUTER, input)) * Math.signum(input)));
    }

    public static double getRotDriveInput(){
        double speedLimitFactor = getSpeedMultiplier() * RobotContainer.drivetrain.rotationSpeedMultiplier.getDouble(1);

        double rotInput = driverController.getRightX()*speedLimitFactor;
        rotInput = Math.abs(rotInput) > 0.1 ? rotInput*0.5 : 0;
        return rotInput;
    }

    public static double getSpeedMultiplier(){
        double speedLimitFactor = RobotContainer.drivetrain.speedLimitFactor.getDouble(1.0);
        double defaultSpeed = RobotContainer.drivetrain.defaultSpeedFactor.getDouble(0.5);
        double slowdownMultiplier = slowDriveButton.getAsBoolean() ? 0.2 : 1;
        return Utils.Lerp(defaultSpeed, speedLimitFactor, getRightTriggerInput()) * slowdownMultiplier;
    }

    public static double getRightTriggerInput(){
        return driverController.getRightTriggerAxis();
    }
    /**
     * Inner class containing controller bindings
     */
    private static class DriverBindings {
        /** Button to re-zero gyro */
        static final Button ZERO_GYRO = XboxController.Button.kBack;
        /** Button to drive at reduced speed */
        static final Button SLOW_DRIVE_BUTTON = XboxController.Button.kY;
        /** Button to shoot note */
        static final Button SHOOT_BUTTON = XboxController.Button.kRightBumper;
        /** Button for amp shot */
        static final Button AMP_BUTTON = XboxController.Button.kA;
        /** Button for auto intake */
        static final Button AUTO_INTAKE_BUTTON = XboxController.Button.kLeftBumper;
        /** Button toggling constant turn to speaker */
        static final Button TOGGLE_TURN = XboxController.Button.kX;
    }

    private static class OperatorBindings{
        /** Button to spit out note */
        static final Button UNSTUCK_BUTTON = XboxController.Button.kA;
        /** Button to manually intake note */
        static final Button INTAKE_BUTTON = XboxController.Button.kLeftBumper;
        // Passing notes manually acroos the feild 
        static final Button PASSING_BUTTON = XboxController.Button.kB;
        /** Button for auto intake */
        static final Button SPINUP_SHOOTER_BUTTON = XboxController.Button.kRightBumper;
        /** Button to extend climber */
        static final Button EXTEND_CLIMB_BUTTON = XboxController.Button.kX;
        /** Button to retract climber */
        static final Button RETRACT_CLIMB_BUTTON = XboxController.Button.kY;
        /** Button to blindly advance intake */
        static final Button ADVANCE_INTAKE_BUTTON = XboxController.Button.kStart;
    }

    /** Port for controller used by driver */
    private static final int DRIVER_CONTROLLER_PORT = 0;
    /** Port for controller used by operator */
    private static final int OPERATOR_CONTROLLER_PORT = 1;


    /** Controller used by driver, mapped to {@link #DRIVER_CONTROLLER_PORT} */
    public static final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
    /** Controller used by driver, mapped to {@link #OPERATOR_CONTROLLER_PORT} */
    public static final XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);


    /** Zero gyro button. Mapped to {@link Bindings#ZERO_GYRO} */
    public static final JoystickButton zeroButton = new JoystickButton(driverController, DriverBindings.ZERO_GYRO.value);
    /** Drive reduced speed button. Mapped to {@link Bindings#SLOW_DRIVE_BUTTON} */
    public static final JoystickButton slowDriveButton = new JoystickButton(driverController, DriverBindings.SLOW_DRIVE_BUTTON.value);

    /**Button to unstuck note */
    public static final JoystickButton unstuckButton = new JoystickButton(operatorController, OperatorBindings.UNSTUCK_BUTTON.value);
    /**Button for passing notes */
    public static final JoystickButton passingAcrossFieldButton = new JoystickButton(operatorController, OperatorBindings.PASSING_BUTTON.value);

    /**Button to trigger intake */
    public static final JoystickButton intakeButton = new JoystickButton(operatorController, OperatorBindings.INTAKE_BUTTON.value);
    /**Button to run auto intake */
    public static final JoystickButton autoIntakeButton = new JoystickButton(driverController, DriverBindings.AUTO_INTAKE_BUTTON.value);
    /**Button to trigger shooter */
    public static final JoystickButton speakerShooterButton = new JoystickButton(driverController, DriverBindings.SHOOT_BUTTON.value);
    /**Button to trigger amp shoot */
    public static final JoystickButton ampButton = new JoystickButton(driverController, DriverBindings.AMP_BUTTON.value);

    /**Button to spinup shooter preemptively for shot */
    public static final JoystickButton spinupShooterButton = new JoystickButton(operatorController, OperatorBindings.SPINUP_SHOOTER_BUTTON.value);

    /**Button to extend climb */
    public static final JoystickButton extendClimbButton = new JoystickButton(operatorController, OperatorBindings.EXTEND_CLIMB_BUTTON.value);
    /**Button to extend climb */
    public static final JoystickButton retractClimbButton = new JoystickButton(operatorController, OperatorBindings.RETRACT_CLIMB_BUTTON.value);

    /** Button to blindly advance intake */
    public static final JoystickButton advanceIntakeButton = new JoystickButton(operatorController, OperatorBindings.ADVANCE_INTAKE_BUTTON.value);
   
    /** Button to blindly advance intake */
    public static final JoystickButton constantTurnButton = new JoystickButton(operatorController, OperatorBindings.ADVANCE_INTAKE_BUTTON.value);
}
 