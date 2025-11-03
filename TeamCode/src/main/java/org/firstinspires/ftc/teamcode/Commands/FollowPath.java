package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

public class FollowPath extends CommandBase {
    PathChain pathChain;
    ChassisSubsystem chassis;
    TelemetryManager telemetry;
    public FollowPath(PathChain pathChain, ChassisSubsystem chassis, TelemetryManager telemetry){
        this.pathChain = pathChain;
        this.chassis = chassis;
        this.telemetry = telemetry;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        super.initialize();
        chassis.getFollower().followPath(pathChain);
        telemetry.debug("Following Path, ROBOT IS MOVING");
        follower.update();
        drawCurrent();
    }

    @Override
    public void execute() {
        super.execute();
        follower.update();
        drawCurrentAndHistory();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        telemetry.debug("PATH HAS ENDED, pick up the controller");
    }

    @Override
    public boolean isFinished() {
        return !chassis.followerIsBusy();
    }
}
