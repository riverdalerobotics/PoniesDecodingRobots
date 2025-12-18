package org.firstinspires.ftc.teamcode.CompCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultFEILDCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTag;
import org.firstinspires.ftc.teamcode.Commands.CloseShot;
import org.firstinspires.ftc.teamcode.Commands.FeedShooter;
import org.firstinspires.ftc.teamcode.Commands.RevUpToShoot;
import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Timer;
import org.firstinspires.ftc.teamcode.OI;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(group = "Comp", name = "BLUE 1/3 of Celestia")
public class TeleopOneShooterBlue extends CommandOpMode {
    LLsubsystem limelight;
    ShooterSubsystem snap;
    ChassisSubsystem chassis;
    ChassisDefaultFEILDCommand chassisDefault;
    ShooterDefaultCommand snapDefault;
    GamepadEx driver;
    GamepadEx operator;
    TelemetryManager telemetryM;
    RevUpToShoot revShoot;
    VoltageSensor voltage;
    OI oi;

    @Override
    public void initialize(){
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = new LLsubsystem(hardwareMap);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        oi = new OI(driver, operator);
        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
        chassisDefault = new ChassisDefaultFEILDCommand(chassis, telemetryM, driver);
        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
        snapDefault = new ShooterDefaultCommand(snap);
        revShoot = new RevUpToShoot(snap);
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        chassis.initBlue();


        register(snap, chassis);
        schedule(snapDefault, chassisDefault);
        chassis.setDefaultCommand(chassisDefault);
        CommandScheduler.getInstance().setDefaultCommand(snap, snapDefault);
    }

    @Override
    public void run(){

        CommandScheduler.getInstance().run();
        if(driver.isDown(GamepadKeys.Button.START)){
            chassis.resetPos();
        }
        oi.revButton().whenHeld(revShoot);
        oi.shooterButton().whenPressed(
                new ParallelDeadlineGroup(
                        new Timer(RobotConstants.Teleop.HOLD_THE_ARM),
                        new FeedShooter(snap))
        );
        oi.pointAtAT().whenHeld(
                new ChassisLookToAprilTag(chassis, limelight, telemetryM, 5, driver)

        );
        oi.closeShot().whenPressed(



                                new CloseShot(snap)
                )

        ;

        telemetryM.addData("angle", snap.getHoodAngle());
        telemetryM.addData("yaw", chassis.yawPitchRollAngles().getYaw());
        telemetryM.addData("speed", snap.getSpeed());
        telemetryM.addData("Shooter Speed", RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO);
        telemetryM.addData("ta", snap.getLLResult().getTa());
        telemetry.addData("voltage", voltage.getVoltage());
        telemetryM.update();
        telemetry.update();
    }
}
