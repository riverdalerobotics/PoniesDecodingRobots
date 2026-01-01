package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class RumbleCommand extends CommandBase {
    GamepadEx driver;
    ShooterSubsystem shooter;
    Telemetry telemetry;
    public RumbleCommand(GamepadEx gamepad, ShooterSubsystem shooter, Telemetry telemetry){
        this.driver = gamepad;
        this.telemetry = telemetry;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        super.execute();
        if(shooter.getShooterPID().atSetPoint()){
            driver.gamepad.rumble(100);
            telemetry.addLine("YAYAYAYA");
        }else{
            driver.gamepad.stopRumble();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driver.gamepad.stopRumble();
    }
}
