package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class ChassisSubsystem extends SubsystemBase {
    TelemetryManager telemetry;
    //Limelight3A limelight;
    MotorEx fl, fr, bl, br;
    Follower follower;
    MecanumDrive drive;
    SparkFunOTOS otos;//edit
    LLResult LLresults;
    public int id;
    public Pose currentPos = new Pose(0, 0, 0);

    public ChassisSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        // limelight = new LLsubsystem(hardwareMap).limelight;
        fl = new MotorEx(hardwareMap, RobotConstants.Hardware.FRONT_LEFT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        fr = new MotorEx(hardwareMap, RobotConstants.Hardware.FRONT_RIGHT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        br = new MotorEx(hardwareMap, RobotConstants.Hardware.BACK_RIGHT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        bl = new MotorEx(hardwareMap, RobotConstants.Hardware.BACK_LEFT_MOTOR, RobotConstants.Hardware.DRIVE_MOTOR_TYPE);
        otos = hardwareMap.get(SparkFunOTOS.class, RobotConstants.Hardware.OTOS_SENSOR);
        fl.setInverted(true);
        bl.setInverted(true);
        this.drive = new MecanumDrive(fl, fr, bl, br);
        this.telemetry = telemetry;
        this.follower = Constants.createFollower(hardwareMap);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(RobotConstants.Hardware.OTOS_OFFSET);
//        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//        limelight.start();
//        LLresults = limelight.getLatestResult();
    }
    public void resetPos(){
        otos.begin();
        otos.setPosition(new SparkFunOTOS.Pose2D(0,0,0));
    }
    public Pose getPose() {
        return new Pose(otos.getPosition().x, otos.getPosition().y, otos.getPosition().h*6);
    }

    public Pose3D getPoseLL() {
        if (LLresults.isValid()) {
            return LLresults.getBotpose_MT2();
        } else {
            return null;
        }
    }

    public LLResult getLLresults() {
        return LLresults;
    }

    public Pose getPoseLLAsPose2D() {
        return new Pose(-(getPoseLL().getPosition().y + 72) / 39.3701,
                -(getPoseLL().getPosition().x + 72) / 39.3701,
                getPose().getHeading());
    }

    //    public void selectPipeline(int pipeline){
//        limelight.pipelineSwitch(pipeline);
//    }
    public void driveRobotOriented(double strafeSpeed, double forwardSpeed, double turn) {
        drive.driveRobotCentric(strafeSpeed, turn, forwardSpeed, true);
    }

    public void driveFieldOriented(double strafeSpeed, double forwardSpeed, double turn) {
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turn, getPose().getHeading(), false);
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

    //    public LLResult getLLResult(){
//        return limelight.getLatestResult();
//    }
    @Override
    public void periodic() {
//        limelight.updateRobotOrientation(getPose().getHeading());
        super.periodic();
        currentPos = getPose();
        follower.update();
        telemetry.debug("Position otos: ", otos.getPosition());
        telemetry.debug("Position: ", getPose());
//        telemetry.debug("Position Using LL: ", getPoseLL());
        follower.setPose(currentPos);
    }

}