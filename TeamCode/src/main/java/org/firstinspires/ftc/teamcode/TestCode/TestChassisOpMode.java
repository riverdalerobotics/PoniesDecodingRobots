package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultFEILDCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultROBOTCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

@TeleOp(group = "Test", name = "Test Chassis op mode")
public class TestChassisOpMode extends CommandOpMode {
    ChassisSubsystem chassis;
//    DcMotor shooter;
    GamepadEx driver;
    TelemetryManager telemetryM;
//    ShooterDefaultCommand shooterDefault;
    ChassisDefaultFEILDCommand chassisDefaultCommand;
//    ShooterSubsystem shooterSubsystem;
//    LLsubsystem limelight;

    @Override
    public void initialize() {
//        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
        driver = new GamepadEx(gamepad1);
//        limelight = new LLsubsystem(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
        chassisDefaultCommand = new ChassisDefaultFEILDCommand(chassis, telemetryM, driver);
//        shooterDefault = new ShooterDefaultCommand(shooterSubsystem, driver);
        register(chassis);
        schedule(chassisDefaultCommand);
        chassis.setDefaultCommand(chassisDefaultCommand);
//        shooterSubsystem.setDefaultCommand(shooterDefault);

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        super.run();
//       Button shoot = new GamepadButton(
//               driver, GamepadKeys.Button.A
//       ).whenHeld(
//               new RevUpToShoot(shooterSubsystem)
//       );
//        Button pointAtAT = new GamepadButton(
//                driver, GamepadKeys.Button.A
//        ).whenPressed(
//                new ChassisLookToAprilTag(chassis, limelight, telemetryM, 0, driver)
//        );
        telemetryM.addData("yaw", chassis.yawPitchRollAngles().getYaw());
        telemetryM.update();
    }
}
