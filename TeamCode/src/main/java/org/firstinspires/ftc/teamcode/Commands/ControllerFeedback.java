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
                .addStep(1, 0, 0, 1000) // Show red for 250ms
                .addStep(1, 1, 1, 1000) // Show white for 250ms
                .build();
        driver.gamepad.runLedEffect(rgbEffect);
    }

    @Override
    public void execute() {
        super.execute();

        if(snap.getShooterPID().atSetPoint()&&crackle.getShooterPID().atSetPoint()&&pop.getShooterPID().atSetPoint()){
            driver.gamepad.rumble(800);
            driver.gamepad.setLedColor(0, 1, 0, 1000);
            telemetry.addLine("YAYAYAYA");
        }else{
            driver.gamepad.stopRumble();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driver.gamepad.stopRumble();
        driver.gamepad.setLedColor(0,0,1, 100000000);
    }
}
