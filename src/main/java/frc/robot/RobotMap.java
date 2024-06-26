package frc.robot;

/**
 * This class cointains definitions for the robot's hardware devices.
 * These include components such as motor controllers and soleniods used by the
 * subsystems.
 * TODO: Change CanIDS
 */
public class RobotMap {

    /**
     * Inner class containing CANIDs
     */
    public static class CANID {
        /** CAN ID for front-left drive falcon */
        public static final int FL_DRIVE_FALCON = 3;
        /** CAN ID for front-left steer falcon */
        public static final int FL_STEER_FALCON = 2;
        /** CAN ID for front-left steer encoder */
        public static final int FL_STEER_ENCODER = 10;
        /** CAN ID for front-right drive falcon */
        public static final int FR_DRIVE_FALCON = 5;
        /** CAN ID for front-right steer falcon */
        public static final int FR_STEER_FALCON = 4;
        /** CAN ID for front-left steer encoder */
        public static final int FR_STEER_ENCODER = 11;
        /** CAN ID for back-left drive falcon */
        public static final int BL_DRIVE_FALCON = 7;
        /** CAN ID for back-left steer falcon */
        public static final int BL_STEER_FALCON = 6;
        /** CAN ID for front-left steer encoder */
        public static final int BL_STEER_ENCODER = 12;
        /** CAN ID for back-right drive falcon */
        public static final int BR_DRIVE_FALCON = 9;
        /** CAN ID for back-right steer falcon */
        public static final int BR_STEER_FALCON = 8;
        /** CAN ID for front-left steer encoder */
        public static final int BR_STEER_ENCODER  = 13;
        /** CAN ID for CTR Pigeon Gyro */
        public static final int PIGEON = 14;
        /** CAN ID for cassette intake motor 1 */
        public static final int IN1_CASSETTE = 20;
        /** CAN ID for cassette intake motor 2*/
        public static final int IN2_CASSETTE = 21;
        /** CAN ID for cassette shoot motor left */
        public static final int L_OUT_CASSETTE = 25;
        /** CAN ID for cassette shoot motor left */
        public static final int R_OUT_CASSETTE = 26;
        /** CAN ID for cassette effector motor left */
        public static final int EFFECTOR_MOTOR = 30;
        /** CAN ID for effector CANcoder */
        public static final int EFFECTOR_CAN_CODER = 31;
        /** CAN ID for climber motor */
        public static final int CLIMB_MOTOR = 18;
    }

    public static class PneumaticsChannel {}

    public static class PWMPorts {
        
          public static final int LED_STRIP = 4;
        public static final int CAMERA_SERVO_ID = 6;
    }
      
    public static class DIOPorts {

        public static final int DIO_IntakeSensor = 1;

    }
    
    
    /**
     * Function to initialise hardware
     * Should be called in {@link Robot#robotInit()}
     */
    public static void Init() {}
}