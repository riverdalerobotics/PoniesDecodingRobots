package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ChassisAutoMoveUsingTime;
import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultROBOTCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisLLAutoDrive;
import org.firstinspires.ftc.teamcode.Commands.ChassisLLAutoTurn;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTag;
import org.firstinspires.ftc.teamcode.Commands.LimelightSeesAT;
import org.firstinspires.ftc.teamcode.Commands.ShootSequence;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;

@TeleOp(group = "Test", name = "Test Chassis op mode")
public class TestPointAtAT extends CommandOpMode {
    ChassisSubsystem chassis;
//    DcMotor shooter;
    GamepadEx driver;
    TelemetryManager telemetryM;
//    ShooterDefaultCommand shooterDefault;
    ChassisDefaultROBOTCommand chassisDefaultCommand;
//    ShooterSubsystem shooterSubsystem;
    LLsubsystem limelight;

    @Override
    public void initialize() {
//        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
        driver = new GamepadEx(gamepad1);
        limelight = new LLsubsystem(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
        chassisDefaultCommand = new ChassisDefaultROBOTCommand(chassis, telemetryM, driver);
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
        if(gamepad1.start){

        }
        Button pointAtAT = new GamepadButton(
                driver, GamepadKeys.Button.A
        ).whileHeld(
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new LimelightSeesAT(limelight),
                                new ChassisAutoMoveUsingTime(chassis, telemetryM, 0.5, 0, 0)
                        ),
                        new ChassisLLAutoDrive(chassis, limelight, telemetry, new Pose(0, 0, 45)),
                        new ChassisLLAutoTurn(chassis, limelight, telemetry, 90)
                )

        );
        telemetry.addData("position", limelight.getLLResults().getBotpose().getPosition());
        telemetry.update();
        telemetryM.update();
    }
}
