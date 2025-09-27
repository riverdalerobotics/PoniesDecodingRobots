package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

public class ChassisMoveToPosition extends CommandBase {
    private final ChassisSubsystem chassisSubsystem;
    PathBuilder pathBuilder;
    PathChain pathChain;
    Follower follower;
    Pose startPos;
    Pose targetPos;

    public ChassisMoveToPosition(ChassisSubsystem subsystem, Follower follower, PathBuilder pathBuilder, Pose startPos, Pose targetPos){
        this.follower = follower;
        this.pathBuilder = pathBuilder;
        this.startPos = startPos;
        chassisSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        super.initialize();
        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(startPos, targetPos))
                .setLinearHeadingInterpolation(startPos.getHeading(), targetPos.getHeading())
                .build();

        follower.followPath(pathChain);
    }
    @Override
    public void execute(){
        super.execute();
        follower.update();
    }
}
