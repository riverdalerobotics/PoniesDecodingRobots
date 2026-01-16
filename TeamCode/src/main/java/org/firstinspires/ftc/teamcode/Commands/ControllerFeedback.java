package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ControllerFeedback extends CommandBase {
    GamepadEx driver;
    ShooterSubsystem snap, crackle, pop;
    Telemetry telemetry;
    Gamepad.LedEffect rgbEffect;
    public ControllerFeedback(GamepadEx gamepad, ShooterSubsystem snap, ShooterSubsystem crackle, ShooterSubsystem pop, Telemetry telemetry){
        this.driver = gamepad;
        this.telemetry = telemetry;
        this.snap = snap;
        this.crackle = crackle;
        this.pop = pop;
        rgbEffect = new Gamepad.LedEffect.Builder()
                .addStep(1, 0, 0, 500) // Show red for 250ms
                .addStep(1, 1, 1, 500) // Show white for 250ms
                .build();
    }

    @Override
    public void execute() {
        super.execute();
        if(snap.getShooterPID().atSetPoint()&&crackle.getShooterPID().atSetPoint()&&pop.getShooterPID().atSetPoint()){
            driver.gamepad.rumble(100);
            driver.gamepad.setLedColor(0, 1, 0, 100);
            telemetry.addLine("YAYAYAYA");
        }else{
            driver.gamepad.stopRumble();
            driver.gamepad.runLedEffect(rgbEffect);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driver.gamepad.stopRumble();
    }
}
