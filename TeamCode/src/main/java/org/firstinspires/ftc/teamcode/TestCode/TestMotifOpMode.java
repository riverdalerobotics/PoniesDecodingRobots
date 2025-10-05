package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
@TeleOp(group = "Test", name = "Motif Test")
public class TestMotifOpMode extends CommandOpMode {

    ShooterSubsystem snap, crackle, pop;
    IntakeSubsystem intake;
    ShooterDefaultCommand snapDefault, crackleDefault, popDefault;
    GamepadEx operator = new GamepadEx(gamepad2);
    TelemetryManager telemetryM;
    char[] motif = {'p', 'p', 'g'};

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new IntakeSubsystem(hardwareMap, telemetryM);
        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
        pop = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.POP);
        crackle  = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.CRACKLE);
        snapDefault = new ShooterDefaultCommand(snap);
        crackleDefault = new ShooterDefaultCommand(crackle);
        popDefault = new ShooterDefaultCommand(pop);


        register(snap, crackle, pop);
        schedule(snapDefault, crackleDefault, popDefault);

        snap.setDefaultCommand(snapDefault);
        crackle.setDefaultCommand(crackleDefault);
        pop.setDefaultCommand(popDefault);

    }

    @Override
    public void run() {
        super.run();
        snap.colour = intake.whatBallsDoWeHave()[0];
        crackle.colour = intake.whatBallsDoWeHave()[1];
        pop.colour = intake.whatBallsDoWeHave()[2];
        Button shooterButton = new GamepadButton(
                operator, GamepadKeys.Button.RIGHT_BUMPER
        );


    }
}
