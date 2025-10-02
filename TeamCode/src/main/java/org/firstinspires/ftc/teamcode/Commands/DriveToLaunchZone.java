package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

public class DriveToLaunchZone extends CommandBase {


    char teamColour;
    ChassisSubsystem chassis;
    Pose target;


    public DriveToLaunchZone(char colour, ChassisSubsystem chassis){
        this.teamColour = colour;
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        super.initialize();
        chassis.stop();
        double x = (chassis.getPoseLL().getPosition().x-chassis.getPoseLL().getPosition().y)/2;
        double y;
        if(teamColour == 'r'){
            y = -x;
            target = new Pose(x, y, Math.toRadians(48));
        } else{
            y = x;
            target = new Pose(x, y, Math.toRadians(132));
        }
        if(y>0){
            y=0;
        }

    }

    @Override
    public void execute() {
        double x = (chassis.getPoseLL().getPosition().x-chassis.getPoseLL().getPosition().y)/2;
        double y;
        if(teamColour == 'r'){
            y = -x;
            target = new Pose(x, y, Math.toRadians(48));
        } else{
            y = x;
            target = new Pose(x, y, Math.toRadians(132));
        }
        if(y>0){
            y=0;
        }
        super.execute();
        chassis.followPath(
                new Path(new BezierCurve(
                    chassis.getPoseLLAsPose2D(), target
                ))
        );

    }

    @Override
    public boolean isFinished() {
            return !chassis.followerIsBusy();
    }
}
