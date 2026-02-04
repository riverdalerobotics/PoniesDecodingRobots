package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Constants file for the entire robot system
 * All tunable parameters are centralized here for easy adjustment
 */
@Configurable
public class RobotConstants {

    // === HARDWARE CONFIGURATION ===
    @Configurable
    public static   class Hardware {
        // Sensor names
        public static SparkFunOTOS.Pose2D OTOS_OFFSET = new SparkFunOTOS.Pose2D(0,0,0);
        public static   String OTOS_SENSOR = "sensor_otos";
        public static   String LIME_LIGHT = "limelight";
        public static   String[] COLOUR_SENSORS = {"LeftColourSensor", "MidColourSensor", "RightColourSensor"};
        // Motor names in hardware configuration
        public static   String FRONT_LEFT_MOTOR = "frontLeft";
        public static   String FRONT_RIGHT_MOTOR = "frontRight";
        public static   String BACK_LEFT_MOTOR = "backLeft";
        public static   String BACK_RIGHT_MOTOR = "backRight";
        public static   String INTAKE_MOTOR = "Intake Motor";
        public static   String[] SNAP = {"Snap Motor", "Snap Hood", "Snap feeder", "F", COLOUR_SENSORS[0], "T"};
        public static   String[] CRACKLE = {"Crackle Motor", "Crackle Hood", "Crackle feeder", "F", COLOUR_SENSORS[1], "F"};
        public static   String[] POP = {"Pop Motor", "Pop Hood", "Pop feeder", "T", COLOUR_SENSORS[2], "F"};



        // Motor specifications
        public static   Motor.GoBILDA DRIVE_MOTOR_TYPE = Motor.GoBILDA.RPM_435;
        public static   Motor.GoBILDA INTAKE_MOTOR_TYPE = Motor.GoBILDA.RPM_435;
        public static   double MOTOR_TICKS_PER_REV = 537.7; // GoBILDA 5202 series

        //Gear Ratios
        public static   double HOOD_SERVO_GEAR_RATIO = 0d;
        public static   double SHOOTER_WHEEL_GEAR_RATIO = 2100;
        public static double SHOOTER_WHEEL_CONVERSION = 0.43;

        // Motor directions (adjust based on your robot)
        public static   boolean FRONT_LEFT_REVERSED = false;
        public static   boolean FRONT_RIGHT_REVERSED = true;
        public static   boolean BACK_LEFT_REVERSED = false;
        public static   boolean BACK_RIGHT_REVERSED = true;

        //Limelight Pipelines
        public static   int APRIL_TAG_PIPELINE = 0;
        public static   int INTAKE_PIPELINE = 1;

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
    @Configurable
    public static   class Physical {
        // Robot dimensions (inches)
        public static   double ROBOT_LENGTH = 18.0;  // Front to back
        public static   double ROBOT_WIDTH = 18.0;   // Left to right
        public static   double ROBOT_HEIGHT = 18.0;  // Bottom to top

        // Drive train specifications
        public static   double WHEEL_DIAMETER = 4.0;     // inches
        public static   double WHEEL_BASE = 14.0;        // Distance between front and rear wheels
        public static   double TRACK_WIDTH = 16.0;       // Distance between left and right wheels
        public static   double GEAR_RATIO = 1.0;         // Adjust if using gear reduction

        // Calculated values
        public static   double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public static   double TICKS_PER_INCH = Hardware.MOTOR_TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

        // Robot mass and inertia (for advanced control)
        public static   double ROBOT_MASS = 15.0;        // kg
        public static   double INERTIA_TENSOR = 10.0;    // kg*m^2
    }

    // === OTOS SENSOR CONFIGURATION ===
    @Configurable
    public static   class OTOS {
        // OTOS sensor offset from robot center (inches)
        // Positive X is forward, positive Y is left, positive heading is CCW
        public static   double X_OFFSET = 0.0;
        public static   double Y_OFFSET = 0.0;
        public static   double H_OFFSET = 0.0; // degrees

        // OTOS scaling factors - TUNE THESE DURING CALIBRATION
        public static   double LINEAR_SCALAR = 1.0;   // Adjust if distance readings are incorrect
        public static   double ANGULAR_SCALAR = 1.0;  // Adjust if angle readings are incorrect

        // Calibration timeout
        public static   double CALIBRATION_TIMEOUT = 5.0; // seconds
    }

    // === PEDRO PATHING CONFIGURATION ===
    @Configurable
    public static   class Pedro {

        // === PRIMARY TUNING PARAMETERS ===
        // Start with these values and adjust based on testing

        // Maximum power settings
        public static   double MAX_POWER = 0.8;           // Maximum motor power (0-1)
        public static   double AUTO_MAX_POWER = 0.9;      // Higher power for autonomous

        // Translational PID - Controls position following accuracy
        public static   double TRANSLATIONAL_PID_KP = 0.15;  // Position error correction strength
        public static   double TRANSLATIONAL_PID_KI = 0.0;   // Integral term (usually 0)
        public static   double TRANSLATIONAL_PID_KD = 0.01;  // Derivative damping

        // Heading PID - Controls rotational accuracy
        public static   double HEADING_PID_KP = 3.0;         // Heading correction strength
        public static   double HEADING_PID_KI = 0.0;         // Integral term (usually 0)
        public static   double HEADING_PID_KD = 0.1;         // Rotational damping

        // === ADVANCED TUNING PARAMETERS ===

        // Drive vector scaling - Controls path following aggression
        public static   double DRIVE_VECTOR_SCALAR = 1.0;    // Overall drive strength multiplier

        // Zero power acceleration - Deceleration when motors get no power
        public static   double FORWARD_ZERO_POWER_ACCELERATION = -4.0;   // in/s² (forward/backward)
        public static   double LATERAL_ZERO_POWER_ACCELERATION = -10.0;  // in/s² (left/right)

        // Centripetal force scaling - How aggressively robot takes curves
        public static   double CENTRIPETAL_FORCE_SCALAR = 0.0002;  // Higher = sharper curves

        // Path following tolerances
        public static   double POSITION_TOLERANCE = 1.0;     // inches - how close to target point
        public static   double HEADING_TOLERANCE = 2.0;      // degrees - how close to target heading
        public static   double VELOCITY_TOLERANCE = 5.0;     // in/s - how slow robot must be

        // Timeout settings
        public static   double PATH_TIMEOUT = 30.0;          // seconds - max time for any path
        public static   double POINT_TIMEOUT = 10.0;         // seconds - max time to reach a point
    }

    // === TELEOP CONTROL SETTINGS ===
    @Configurable
    public static   class Teleop {
        // Joystick deadzone
        public static   double DRIVE_DEADZONE = 0.1;
        public static   double TURN_DEADZONE = 0.1;

        // Drive scaling
        public static   double NORMAL_DRIVE_SCALE = 1.0;
        public static   double PRECISION_DRIVE_SCALE = 0.3;   // When precision button held
        public static   double TURN_SCALE = 0.8;              // Turn sensitivity

        // Control response curves (1.0 = linear, >1.0 = more aggressive)
        public static   double DRIVE_EXPO = 1.5;
        public static   double TURN_EXPO = 2.0;

        // Field-centric default setting
        public static   boolean DEFAULT_FIELD_CENTRIC = true;

        //Setpoints
        public static   double INTAKE_X_ANGLE_CHASSIS = 0d;
        public static   double INTAKE_Y_ANGLE_CHASSIS = 0d;
        public static   double SHOOTER_INTAKE_SPEED = -0.3;
        public static   double INTAKE_SPEED = -1;
        public static double FEED = 0.2;
        public static double STOW_FEEDER = 0;
        public static double INTAKE_FEEDER = 0.1;

        //Timers
        public static long SHOOTER_TIMER = 1000;
        public static long HOLD_THE_ARM = 500;
        public static long DRIVE_FORWARD_AUTO = 1000;
        public static long DRIVE_FORWARD_CLOSE_AUTO = 500;


        //The rest
        public static   double[] WHITE_RGB = {0, 0, 0};
        public static   double INTAKE_MIN = 100;
        public static double CLOSE_SHOT_THRESHOLD = 1;
        public static double CLOSE_SHOT = 1585;
        public static double VERY_CLOSE_SHOT = 500;
        public static double FAR_SHOT = 1930;
        public static   double WHITE_THRESHOLD = 20;
        public static double[] FAR_SHOT_SPEEDS_VOLT = {0.911, 0.925, 0.95, 0.97};
                //{0.83, 0.91, 0.935, 0.94};
        public static double[] CLOSE_SHOT_SPEEDS_VOLTS = {0.836, 0.861, 0.866, 0.886};
                        //{0.8, 0.85, 0.88, 0.9};
    }

    // === AUTONOMOUS SETTINGS ===
    @Configurable
    public static   class Autonomous {
        // Default autonomous speeds
        public static   double DEFAULT_DRIVE_SPEED = 0.6;
        public static   double DEFAULT_TURN_SPEED = 0.4;
        public static   double PRECISE_DRIVE_SPEED = 0.3;

        // Safety timeouts
        public static   double MAX_AUTO_TIME = 30.0;         // seconds
        public static   double ACTION_TIMEOUT = 5.0;         // seconds for individual actions

        // Position tolerances for autonomous movements
        public static   double AUTO_POSITION_TOLERANCE = 0.5;  // inches
        public static   double AUTO_HEADING_TOLERANCE = 1.0;   // degrees
    }

    // === FIELD DIMENSIONS AND POSITIONS ===
    @Configurable
    public static   class Field {
        // FTC field dimensions (inches)
        public static   double FIELD_WIDTH = 144.0;   // 12 feet
        public static   double FIELD_LENGTH = 144.0;  // 12 feet

        // Common field positions (adjust for current game)
        // These are examples - update for your specific game
        public static   class RedAlliance {
            // Starting positions
            public static   double START_X = -36.0;
            public static   double START_Y = -63.0;
            public static   double START_HEADING = Math.PI/2; // Facing forward

            // Scoring positions
            public static   double HIGH_BASKET_X = -58.0;
            public static   double HIGH_BASKET_Y = -58.0;

            // Sample/specimen positions
            public static   double SAMPLE_1_X = -48.0;
            public static   double SAMPLE_1_Y = -45.0;

            // Parking positions
            public static   double PARK_X = -24.0;
            public static   double PARK_Y = -12.0;
        }
        @Configurable
        public static   class BlueAlliance {
            // Mirror red alliance positions
            public static   double START_X = 36.0;
            public static   double START_Y = 63.0;
            public static   double START_HEADING = -Math.PI/2;

            public static   double HIGH_BASKET_X = 58.0;
            public static   double HIGH_BASKET_Y = 58.0;

            public static   double SAMPLE_1_X = 48.0;
            public static   double SAMPLE_1_Y = 45.0;

            public static   double PARK_X = 24.0;
            public static   double PARK_Y = 12.0;
        }
    }
    @Configurable
    public static class Testing{
        public static long time = 2000;
        public static Pose CHASSIS_DRIVE_TO_SECOND_INTAKE = new Pose(-3.4,2., -20);
        public static Pose CHASSIS_DRIVE_TO_FIRST_INTAKE = new Pose(-2.7,2.3, -30);
        public static Pose CHASSIS_DRIVE_TO_SECOND_SHOT = new Pose(-2.4, 2.4, -40);
    }
    // === TUNING AND CALIBRATION ===
    @Configurable
    public static   class Tuning {
        // Test parameters for tuning OpModes
        public static   double DEFAULT_TEST_DISTANCE = 48.0;  // inches
        public static   double DEFAULT_TEST_ANGLE = 90.0;     // degrees

        // Tuning increments
        public static   double PID_TUNING_INCREMENT = 0.01;
        public static   double POWER_TUNING_INCREMENT = 0.05;
        public static   double SCALAR_TUNING_INCREMENT = 0.001;

        // Calibration parameters
        public static   int CALIBRATION_SAMPLES = 100;
        public static   double CALIBRATION_DISTANCE = 48.0;    // inches for straight line test
        public static   double CALIBRATION_ANGLE = 360.0;      // degrees for rotation test

        //PID coefficients
        public static double[] SHOOTER_PIDF_COEFFICIENTS = {0.0023, 0.0075, 0.0005, 0.000435};
        public static   double[] CHASSIS_PID_COEFFICIENTS_POINT = {-0.025, -0.02, -0.001};
        public static   double[] CHASSIS_TURN_PID_COEFFICIENTS = {0.03, 0.034, 0.00004};
        public static   double[] CHASSIS_DRIVE_PID_COEFFICIENTS = {0.75, 0.2, 0.05};
        public static   double [] CHASSIS_TOLERANCE = {0.09, 1};
        public static double[] SHOOTER_TOLERANCE = {30,10};

        //Other coefficients
        public static double POINT_AT_AT_TARGET = 0;
        public static double POINT_AT_AT_TARGET_AUTO = 0;
        public static   double TA_TO_ANGLE = 0.04;
        public static double MAX_ANGLE = 0.15;
        public static double MIN_ANGLE = 0;
    }

    // === SAFETY LIMITS ===
    @Configurable
    public static   class Safety {
        // Maximum allowed speeds
        public static   double MAX_DRIVE_SPEED = 1.0;
        public static   double MAX_TURN_SPEED = 1.0;

        // Emergency stop conditions
        public static   double MAX_ACCELERATION = 50.0;       // in/s²
        public static   double MAX_POSITION_ERROR = 12.0;     // inches before emergency stop
        public static   double MAX_HEADING_ERROR = 45.0;      // degrees before emergency stop

        // Voltage monitoring
        public static   double MIN_BATTERY_VOLTAGE = 11.0;    // volts
        public static   double LOW_BATTERY_WARNING = 11.5;    // volts
    }

    // === TELEMETRY SETTINGS ===
    @Configurable
    public static   class Telemetry {
        // Update rates
        public static   int TELEMETRY_UPDATE_MS = 100;        // milliseconds
        public static   int FAST_TELEMETRY_UPDATE_MS = 50;    // for tuning modes

        // Precision for number display
        public static   int POSITION_DECIMAL_PLACES = 2;
        public static   int ANGLE_DECIMAL_PLACES = 1;
        public static   int VELOCITY_DECIMAL_PLACES = 1;

        // Enable/disable different telemetry sections
        public static   boolean SHOW_MOTOR_TELEMETRY = true;
        public static   boolean SHOW_OTOS_TELEMETRY = true;
        public static   boolean SHOW_PEDRO_TELEMETRY = true;
        public static   boolean SHOW_DEBUG_TELEMETRY = false; // Only enable when debugging
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