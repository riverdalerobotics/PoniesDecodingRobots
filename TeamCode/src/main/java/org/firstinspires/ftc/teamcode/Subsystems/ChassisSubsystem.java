package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class ChassisSubsystem extends SubsystemBase {
    TelemetryManager telemetry;
    PIDController chassisPID;
    //Limelight3A limelight;
    MotorEx fl, fr, bl, br;
    Follower follower;
    MecanumDrive drive;
    SparkFunOTOS otos;//edit
    LLResult LLresults;
    IMU imu;
    HardwareMap hardwareMap;
    public int id;
    public Pose currentPos = new Pose(0, 0, 0);

    public ChassisSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        // limelight = new LLsubsystem(hardwareMap).limelight;
        this.hardwareMap = hardwareMap;
        fl = new MotorEx(hardwareMap, RobotConstants.Hardware.FRONT_LEFT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        br = new MotorEx(hardwareMap, RobotConstants.Hardware.FRONT_RIGHT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        fr = new MotorEx(hardwareMap, RobotConstants.Hardware.BACK_RIGHT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        bl = new MotorEx(hardwareMap, RobotConstants.Hardware.BACK_LEFT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        otos = hardwareMap.get(SparkFunOTOS.class, RobotConstants.Hardware.OTOS_SENSOR);
        fl.setInverted(true);
        bl.setInverted(true);
        chassisPID = new PIDController(RobotConstants.Tuning.CHASSIS_TURN_PID_COEFFICIENTS[0],
                RobotConstants.Tuning.CHASSIS_TURN_PID_COEFFICIENTS[1], RobotConstants.Tuning.CHASSIS_TURN_PID_COEFFICIENTS[2]);
        this.drive = new MecanumDrive(fl, fr, bl, br);
        this.telemetry = telemetry;
        this.follower = Constants.createFollower(hardwareMap);
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setOffset(RobotConstants.Hardware.OTOS_OFFSET);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                //TODO: ENSURE THIS IS FIXED PRE COMP
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
    }
    public void initBlue(){
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

    }
    public void initRed(){
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }

    public void resetPos(){
        imu.resetYaw();
    }

    public YawPitchRollAngles yawPitchRollAngles(){
        return imu.getRobotYawPitchRollAngles();
    }

    public Pose3D getPoseLL() {
        if (LLresults.isValid()) {
            return LLresults.getBotpose_MT2();
        } else {
            return null;
        }
    }
    public PIDController getChassisPID(){
        return chassisPID;
    }
    public LLResult getLLresults() {
        return LLresults;
    }
    public Pose getPose() {
        return new Pose(otos.getPosition().x, otos.getPosition().y, yawPitchRollAngles().getYaw());
    }
    public Pose getPoseLLAsPose2D() {
        return new Pose(-(getPoseLL().getPosition().y + 72) / 39.3701,
                -(getPoseLL().getPosition().x + 72) / 39.3701,
                getPose().getHeading());
    }


    public void driveRobotOriented(double strafeSpeed, double forwardSpeed, double turn) {
        fl = new MotorEx(hardwareMap, RobotConstants.Hardware.FRONT_LEFT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        fr = new MotorEx(hardwareMap, RobotConstants.Hardware.FRONT_RIGHT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        br = new MotorEx(hardwareMap, RobotConstants.Hardware.BACK_RIGHT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        bl = new MotorEx(hardwareMap, RobotConstants.Hardware.BACK_LEFT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        drive = new MecanumDrive(fl, fr, bl, br);
        drive.driveRobotCentric(strafeSpeed, turn, forwardSpeed, false);

    }
    public void driveFieldOriented(double strafeSpeed, double forwardSpeed, double turn) {
        //Robot Oriented
        fl = new MotorEx(hardwareMap, RobotConstants.Hardware.FRONT_LEFT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        br = new MotorEx(hardwareMap, RobotConstants.Hardware.FRONT_RIGHT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        fr = new MotorEx(hardwareMap, RobotConstants.Hardware.BACK_RIGHT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        bl = new MotorEx(hardwareMap, RobotConstants.Hardware.BACK_LEFT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        drive = new MecanumDrive(fl, fr, bl, br);
        fl.setInverted(false);
        bl.setInverted(false);
        drive.driveFieldCentric(strafeSpeed, -forwardSpeed, turn, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), false);
    }
    public void fieldOriented(double strafeSpeed, double forwardSpeed, double turn) {
        //field oriented
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // calculated from IMU
        fl = new MotorEx(hardwareMap, RobotConstants.Hardware.FRONT_LEFT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        br = new MotorEx(hardwareMap, RobotConstants.Hardware.FRONT_RIGHT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        fr = new MotorEx(hardwareMap, RobotConstants.Hardware.BACK_RIGHT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        bl = new MotorEx(hardwareMap, RobotConstants.Hardware.BACK_LEFT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        drive = new MecanumDrive(fl, fr, bl, br);
        fl.setInverted(false);
        bl.setInverted(false);

        double rotX = (strafeSpeed * Math.cos(yaw)) + (forwardSpeed * Math.sin(yaw));
        double rotY = (strafeSpeed * Math.sin(yaw)) - (forwardSpeed * Math.cos(yaw));
        drive.driveRobotCentric(rotX, rotY, turn, false);

    }

    public void stop() {
        drive.stop();
        follower.breakFollowing();
    }

    public boolean followerIsBusy() {
        return follower != null && follower.isBusy();
    }

    public void followPath(Path path) {
        follower.followPath(path);
    }

    public void followPathChain(PathChain pathChain) {
        follower.followPath(pathChain);
    }

    public Follower getFollower() {
        return follower;
    }


    @Override
    public void periodic() {
        super.periodic();
        currentPos = getPose();
        follower.update();
        follower.setPose(currentPos);
    }

}