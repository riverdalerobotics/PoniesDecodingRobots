package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;

@TeleOp(group = "Test", name = "Lime Test")
public class TestLimeLightOpMode extends CommandOpMode {
    LLsubsystem limelight;

    @Override
    public void initialize() {
        limelight = new LLsubsystem(hardwareMap);
    }
    @Override
    public void run() {
        telemetry.addData("pos", limelight.getLLResults());
        telemetry.update();
    }
}
