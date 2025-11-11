package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class HoodTestCommand extends CommandBase{
    ShooterSubsystem shooter;

    TelemetryManager telemetryManager;
    public HoodTestCommand(ShooterSubsystem shooter, TelemetryManager telemetryManager){
        this.shooter = shooter;
        this.telemetryManager = telemetryManager;
        addRequirements(shooter);

    }

    @Override
    public void initialize() {
        super.initialize();
        telemetryManager.addLine("THIS IS INIT");
    }

    @Override
    public void execute() {
        super.execute();
        telemetryManager.addLine("RHOTHOHT");
        if(shooter.getLLResult().isValid()){
            shooter.setHoodAngle(RobotConstants.clamp(RobotConstants.Tuning.TA_TO_ANGLE*shooter.getLLResult().getTa(), 0, 0.1));
        } else{
            shooter.setHoodAngle(0.1);
        }


    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
