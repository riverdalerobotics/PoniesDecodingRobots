package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;

@TeleOp(group = "Test", name = "Lime Test")
public class TestLimeLightOpMode extends CommandOpMode {
    LLsubsystem limelight;
    TelemetryManager telemetryManager;
    int id = 0;

    @Override
    public void initialize() {
        limelight = new LLsubsystem(hardwareMap);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }
    @Override
    public void run() {

        for(LLResultTypes.FiducialResult fiducialResult : limelight.fiducialResult()){
            id = fiducialResult.getFiducialId();
        }
        telemetryManager.addData("pos", limelight.getLLResults().getBotpose());
        telemetryManager.debug("X angle", limelight.getLLResults().getTx());
        telemetryManager.debug("Y angle", limelight.getLLResults().getTy());
        telemetryManager.debug("Ta", limelight.getLLResults().getTa());
        telemetryManager.debug("ID", id);
        telemetryManager.update();
    }
}
