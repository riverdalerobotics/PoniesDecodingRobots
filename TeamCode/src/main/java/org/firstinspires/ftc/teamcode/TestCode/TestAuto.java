//package org.firstinspires.ftc.teamcode.TestCode;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Commands.ChassisAutoMoveUsingTime;
//import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultAuto;
//import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTag;
//import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTagInAutoBlue;
//import org.firstinspires.ftc.teamcode.Commands.ShootSequence;
//import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
//import org.firstinspires.ftc.teamcode.Commands.Timer;
//import org.firstinspires.ftc.teamcode.RobotConstants;
//import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
//import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;
//import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
//
//@Autonomous(group = "Test", name = "Test Auto not case")
//public class TestAuto extends CommandOpMode {
//    ChassisSubsystem chassis;
//    ShooterSubsystem snap;
//    ChassisDefaultAuto chassisDefault;
//    ShooterDefaultCommand snapDefault;
//    TelemetryManager telemetryM;
//    LLsubsystem limelight;
//    @Override
//    public void initialize() {
//        limelight = new LLsubsystem(hardwareMap);
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
//        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
//        snapDefault = new ShooterDefaultCommand(snap);
//        chassisDefault = new ChassisDefaultAuto(chassis, telemetryM);
//        register(chassis, snap);
//        schedule(snapDefault, chassisDefault);
//        snap.setDefaultCommand(snapDefault);
//        chassis.setDefaultCommand(chassisDefault);
//        schedule(
//                new SequentialCommandGroup(
//                        new ChassisLookToAprilTagInAutoBlue(chassis, limelight, telemetryM, 0),
//                        new ParallelDeadlineGroup(
//                                new SequentialCommandGroup(
//                                        new ShootSequence(snap),
//                                        new ShootSequence(snap),
//                                        new ShootSequence(snap),
//                                        new ShootSequence(snap)
//                                        ),
//                                new ChassisLookToAprilTag(chassis, limelight, telemetryM, 0, new GamepadEx(gamepad1))
//
//                        ),
//
//                        new ParallelDeadlineGroup(new Timer(RobotConstants.Teleop.DRIVE_FORWARD_AUTO), new ChassisAutoMoveUsingTime(chassis, telemetryM, .3, 0, 0))
//                ));
//        chassis.resetPos();
//        CommandScheduler.getInstance().run();
//    }
//
////    @Override
////    public void run() {
////        super.run();
////
////    }
//}
