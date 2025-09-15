package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ChassisSubsystem extends SubsystemBase {
    MotorEx fl, fr, bl, br;
    Follower follower;
    MecanumDrive drive;
    SparkFunOTOS otos;//edit
    public Pose currentPos = new Pose(0, 0, 0);
    public ChassisSubsystem(HardwareMap hardwareMap, PanelsTelemetry telemetry){
        drive = new MecanumDrive(fl, fr, bl, br);
    }
    public Pose getPose(){
        return new Pose(otos.getPosition().x, otos.getPosition().y, Math.toRadians(otos.getPosition().h));
    }
    public void driveRobotOriented(double strafeSpeed, double forwardSpeed, double turn){
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turn, true);
    }
    public void driveFieldOriented(double strafeSpeed, double forwardSpeed, double turn){
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turn, Math.toDegrees(getPose().getHeading()), true);
    }
    public void stop(){
        drive.stop();
        follower.breakFollowing();
    }

    public void followPath(Path path){
        follower.followPath(path);
    }

    public void followPathChain(PathChain pathChain){
        follower.followPath(pathChain);
    }

    public void periodic(){
        currentPos = getPose();
        follower.update();

        follower.setPose(currentPos);
    }

}
