package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.gamepad.Gamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(name="test velo op", group = "Test")
public class TestVeloControlOp extends CommandOpMode {
    ShooterSubsystem snap;
    TelemetryManager telemetryM;
    GamepadEx gamepad;
    double setpoint;


    PIDFController shooterPID;
    @Override
    public void initialize() {
        
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
        this.setpoint = RobotConstants.Teleop.FAR_SHOT;
        this.shooterPID = new PIDFController(RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[0],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[1],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[2],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[3]);
        snap.getShootMotor().motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterPID.setTolerance(RobotConstants.Tuning.SHOOTER_TOLERANCE);
        shooterPID.setSetPoint(setpoint);
    }

    @Override
    public void run() {
        shooterPID.setPIDF(RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[0],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[1],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[2],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[3]);
        double speed = RobotConstants.clamp(shooterPID.calculate(snap.getSpeed(), setpoint),0,1);
        telemetry.addData("set Speed", speed);
        telemetry.addData("The THINGYMAGIC", shooterPID.calculate(snap.getSpeed()));
        telemetry.addData("current Speed", snap.getSpeed());
        snap.setSpeed = speed;
        telemetry.update();
        if(gamepad1.right_bumper){
        snap.getShootMotor().set(speed);
    }else{
            snap.getShootMotor().set(0);
        }
    }
}
