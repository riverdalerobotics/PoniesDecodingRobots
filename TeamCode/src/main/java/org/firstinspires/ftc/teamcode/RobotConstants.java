package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Constants file for the entire robot system
 * All tunable parameters are centralized here for easy adjustment
 */
public final class RobotConstants {

    // === HARDWARE CONFIGURATION ===

    public static final class Hardware {
        // Sensor names
        public static final String OTOS_SENSOR = "sensor_otos";
        public static final String LIME_LIGHT = "limelight";
        public static final String[] COLOUR_SENSORS = {"LeftColourSensor", "MidColourSensor", "RightColourSensor"};
        // Motor names in hardware configuration
        public static final String FRONT_LEFT_MOTOR = "frontLeft";
        public static final String FRONT_RIGHT_MOTOR = "frontRight";
        public static final String BACK_LEFT_MOTOR = "backLeft";
        public static final String BACK_RIGHT_MOTOR = "backRight";
        public static final String INTAKE_MOTOR = "Intake Motor";
        public static final String[] SNAP = {"Snap Motor", "Snap Hood", "Snap feeder", "F", COLOUR_SENSORS[0]};
        public static final String[] CRACKLE = {"Crackle Motor", "Crackle Hood", "Crackle feeder", "T", COLOUR_SENSORS[1]};
        public static final String[] POP = {"Pop Motor", "Pop Hood", "Pop feeder", "T", COLOUR_SENSORS[2]};



        // Motor specifications
        public static final Motor.GoBILDA DRIVE_MOTOR_TYPE = Motor.GoBILDA.RPM_435;
        public static final Motor.GoBILDA INTAKE_MOTOR_TYPE = Motor.GoBILDA.RPM_435;
        public static final double MOTOR_TICKS_PER_REV = 537.7; // GoBILDA 5202 series

        //Gear Ratios
        public static final double HOOD_SERVO_GEAR_RATIO = 0d;
        public static final double SHOOTER_WHEEL_GEAR_RATIO = 0d;

        // Motor directions (adjust based on your robot)
        public static final boolean FRONT_LEFT_REVERSED = false;
        public static final boolean FRONT_RIGHT_REVERSED = true;
        public static final boolean BACK_LEFT_REVERSED = false;
        public static final boolean BACK_RIGHT_REVERSED = true;

        //Limelight Pipelines
        public static final int APRIL_TAG_PIPELINE = 0;
        public static final int INTAKE_PIPELINE = 1;

        //Mechanum thing
        public static MecanumConstants driveConstants = new MecanumConstants()
                .maxPower(1)
                .rightFrontMotorName(RobotConstants.Hardware.FRONT_RIGHT_MOTOR)
                .rightRearMotorName(RobotConstants.Hardware.BACK_RIGHT_MOTOR)
                .leftRearMotorName(RobotConstants.Hardware.BACK_LEFT_MOTOR)
                .leftFrontMotorName(RobotConstants.Hardware.FRONT_LEFT_MOTOR)
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    }

    // === PHYSICAL ROBOT DIMENSIONS ===

    public static final class Physical {
        // Robot dimensions (inches)
        public static final double ROBOT_LENGTH = 18.0;  // Front to back
        public static final double ROBOT_WIDTH = 18.0;   // Left to right
        public static final double ROBOT_HEIGHT = 18.0;  // Bottom to top

        // Drive train specifications
        public static final double WHEEL_DIAMETER = 4.0;     // inches
        public static final double WHEEL_BASE = 14.0;        // Distance between front and rear wheels
        public static final double TRACK_WIDTH = 16.0;       // Distance between left and right wheels
        public static final double GEAR_RATIO = 1.0;         // Adjust if using gear reduction

        // Calculated values
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public static final double TICKS_PER_INCH = Hardware.MOTOR_TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

        // Robot mass and inertia (for advanced control)
        public static final double ROBOT_MASS = 15.0;        // kg
        public static final double INERTIA_TENSOR = 10.0;    // kg*m^2
    }

    // === OTOS SENSOR CONFIGURATION ===

    public static final class OTOS {
        // OTOS sensor offset from robot center (inches)
        // Positive X is forward, positive Y is left, positive heading is CCW
        public static final double X_OFFSET = 0.0;
        public static final double Y_OFFSET = 0.0;
        public static final double H_OFFSET = 0.0; // degrees

        // OTOS scaling factors - TUNE THESE DURING CALIBRATION
        public static final double LINEAR_SCALAR = 1.0;   // Adjust if distance readings are incorrect
        public static final double ANGULAR_SCALAR = 1.0;  // Adjust if angle readings are incorrect

        // Calibration timeout
        public static final double CALIBRATION_TIMEOUT = 5.0; // seconds
    }

    // === PEDRO PATHING CONFIGURATION ===

    public static final class Pedro {

        // === PRIMARY TUNING PARAMETERS ===
        // Start with these values and adjust based on testing

        // Maximum power settings
        public static final double MAX_POWER = 0.8;           // Maximum motor power (0-1)
        public static final double AUTO_MAX_POWER = 0.9;      // Higher power for autonomous

        // Translational PID - Controls position following accuracy
        public static final double TRANSLATIONAL_PID_KP = 0.15;  // Position error correction strength
        public static final double TRANSLATIONAL_PID_KI = 0.0;   // Integral term (usually 0)
        public static final double TRANSLATIONAL_PID_KD = 0.01;  // Derivative damping

        // Heading PID - Controls rotational accuracy
        public static final double HEADING_PID_KP = 3.0;         // Heading correction strength
        public static final double HEADING_PID_KI = 0.0;         // Integral term (usually 0)
        public static final double HEADING_PID_KD = 0.1;         // Rotational damping

        // === ADVANCED TUNING PARAMETERS ===

        // Drive vector scaling - Controls path following aggression
        public static final double DRIVE_VECTOR_SCALAR = 1.0;    // Overall drive strength multiplier

        // Zero power acceleration - Deceleration when motors get no power
        public static final double FORWARD_ZERO_POWER_ACCELERATION = -4.0;   // in/s² (forward/backward)
        public static final double LATERAL_ZERO_POWER_ACCELERATION = -10.0;  // in/s² (left/right)

        // Centripetal force scaling - How aggressively robot takes curves
        public static final double CENTRIPETAL_FORCE_SCALAR = 0.0002;  // Higher = sharper curves

        // Path following tolerances
        public static final double POSITION_TOLERANCE = 1.0;     // inches - how close to target point
        public static final double HEADING_TOLERANCE = 2.0;      // degrees - how close to target heading
        public static final double VELOCITY_TOLERANCE = 5.0;     // in/s - how slow robot must be

        // Timeout settings
        public static final double PATH_TIMEOUT = 30.0;          // seconds - max time for any path
        public static final double POINT_TIMEOUT = 10.0;         // seconds - max time to reach a point
    }

    // === TELEOP CONTROL SETTINGS ===

    public static final class Teleop {
        // Joystick deadzone
        public static final double DRIVE_DEADZONE = 0.1;
        public static final double TURN_DEADZONE = 0.1;

        // Drive scaling
        public static final double NORMAL_DRIVE_SCALE = 1.0;
        public static final double PRECISION_DRIVE_SCALE = 0.3;   // When precision button held
        public static final double TURN_SCALE = 0.8;              // Turn sensitivity

        // Control response curves (1.0 = linear, >1.0 = more aggressive)
        public static final double DRIVE_EXPO = 1.5;
        public static final double TURN_EXPO = 2.0;

        // Field-centric default setting
        public static final boolean DEFAULT_FIELD_CENTRIC = true;

        //Setpoints
        public static final double INTAKE_X_ANGLE_CHASSIS = 0d;
        public static final double INTAKE_Y_ANGLE_CHASSIS = 0d;
        public static final double SHOOTER_SPEED = 0d;
        public static final double INTAKE_SPEED = 0d;

        //Timers
        public static  final long SHOOTER_TIMER = 0;

        //The rest
        public static final double[] WHITE_RGB = {0, 0, 0};
        public static final double INTAKE_MIN = 100;
        public static final double WHITE_THRESHOLD = 20;
    }

    // === AUTONOMOUS SETTINGS ===

    public static final class Autonomous {
        // Default autonomous speeds
        public static final double DEFAULT_DRIVE_SPEED = 0.6;
        public static final double DEFAULT_TURN_SPEED = 0.4;
        public static final double PRECISE_DRIVE_SPEED = 0.3;

        // Safety timeouts
        public static final double MAX_AUTO_TIME = 30.0;         // seconds
        public static final double ACTION_TIMEOUT = 5.0;         // seconds for individual actions

        // Position tolerances for autonomous movements
        public static final double AUTO_POSITION_TOLERANCE = 0.5;  // inches
        public static final double AUTO_HEADING_TOLERANCE = 1.0;   // degrees
    }

    // === FIELD DIMENSIONS AND POSITIONS ===

    public static final class Field {
        // FTC field dimensions (inches)
        public static final double FIELD_WIDTH = 144.0;   // 12 feet
        public static final double FIELD_LENGTH = 144.0;  // 12 feet

        // Common field positions (adjust for current game)
        // These are examples - update for your specific game
        public static final class RedAlliance {
            // Starting positions
            public static final double START_X = -36.0;
            public static final double START_Y = -63.0;
            public static final double START_HEADING = Math.PI/2; // Facing forward

            // Scoring positions
            public static final double HIGH_BASKET_X = -58.0;
            public static final double HIGH_BASKET_Y = -58.0;

            // Sample/specimen positions
            public static final double SAMPLE_1_X = -48.0;
            public static final double SAMPLE_1_Y = -45.0;

            // Parking positions
            public static final double PARK_X = -24.0;
            public static final double PARK_Y = -12.0;
        }

        public static final class BlueAlliance {
            // Mirror red alliance positions
            public static final double START_X = 36.0;
            public static final double START_Y = 63.0;
            public static final double START_HEADING = -Math.PI/2;

            public static final double HIGH_BASKET_X = 58.0;
            public static final double HIGH_BASKET_Y = 58.0;

            public static final double SAMPLE_1_X = 48.0;
            public static final double SAMPLE_1_Y = 45.0;

            public static final double PARK_X = 24.0;
            public static final double PARK_Y = 12.0;
        }
    }

    // === TUNING AND CALIBRATION ===

    public static final class Tuning {
        // Test parameters for tuning OpModes
        public static final double DEFAULT_TEST_DISTANCE = 48.0;  // inches
        public static final double DEFAULT_TEST_ANGLE = 90.0;     // degrees

        // Tuning increments
        public static final double PID_TUNING_INCREMENT = 0.01;
        public static final double POWER_TUNING_INCREMENT = 0.05;
        public static final double SCALAR_TUNING_INCREMENT = 0.001;

        // Calibration parameters
        public static final int CALIBRATION_SAMPLES = 100;
        public static final double CALIBRATION_DISTANCE = 48.0;    // inches for straight line test
        public static final double CALIBRATION_ANGLE = 360.0;      // degrees for rotation test

        //PID coefficients
        public static final double[] CHASSIS_PID_COEFFICIENTS = {0d, 0d, 0d};
        public static final double CHASSIS_TOLERANCE = 0d;

        //Other coefficients
        public static final double TA_TO_ANGLE = 0d;
    }

    // === SAFETY LIMITS ===

    public static final class Safety {
        // Maximum allowed speeds
        public static final double MAX_DRIVE_SPEED = 1.0;
        public static final double MAX_TURN_SPEED = 1.0;

        // Emergency stop conditions
        public static final double MAX_ACCELERATION = 50.0;       // in/s²
        public static final double MAX_POSITION_ERROR = 12.0;     // inches before emergency stop
        public static final double MAX_HEADING_ERROR = 45.0;      // degrees before emergency stop

        // Voltage monitoring
        public static final double MIN_BATTERY_VOLTAGE = 11.0;    // volts
        public static final double LOW_BATTERY_WARNING = 11.5;    // volts
    }

    // === TELEMETRY SETTINGS ===

    public static final class Telemetry {
        // Update rates
        public static final int TELEMETRY_UPDATE_MS = 100;        // milliseconds
        public static final int FAST_TELEMETRY_UPDATE_MS = 50;    // for tuning modes

        // Precision for number display
        public static final int POSITION_DECIMAL_PLACES = 2;
        public static final int ANGLE_DECIMAL_PLACES = 1;
        public static final int VELOCITY_DECIMAL_PLACES = 1;

        // Enable/disable different telemetry sections
        public static final boolean SHOW_MOTOR_TELEMETRY = true;
        public static final boolean SHOW_OTOS_TELEMETRY = true;
        public static final boolean SHOW_PEDRO_TELEMETRY = true;
        public static final boolean SHOW_DEBUG_TELEMETRY = false; // Only enable when debugging
    }

    // === UTILITY METHODS ===

    /**
     * Convert inches to encoder ticks
     */
    public static double inchesToTicks(double inches) {
        return inches * Physical.TICKS_PER_INCH;
    }

    /**
     * Convert encoder ticks to inches
     */
    public static double ticksToInches(double ticks) {
        return ticks / Physical.TICKS_PER_INCH;
    }

    /**
     * Convert degrees to radians
     */
    public static double degreesToRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    /**
     * Convert radians to degrees
     */
    public static double radiansToDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    /**
     * Clamp a value between min and max
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Apply deadzone to joystick input
     */
    public static double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        return value;
    }

    /**
     * Apply exponential curve to input for smoother control
     */
    public static double applyExpo(double input, double expo) {
        return Math.signum(input) * Math.pow(Math.abs(input), expo);
    }

    /**
     * Normalize angle to [-PI, PI] range
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Get the shortest angular distance between two angles
     */
    public static double getAngularDistance(double from, double to) {
        double diff = to - from;
        return normalizeAngle(diff);
    }

    // Prevent instantiation
    private RobotConstants() {
        throw new AssertionError("Constants class should not be instantiated");
    }
}