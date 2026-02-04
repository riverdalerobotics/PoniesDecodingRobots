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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ChassisAutoMoveUsingTime;
import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultFEILDCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisLLAutoDrive;
import org.firstinspires.ftc.teamcode.Commands.ChassisLLAutoTurn;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTagInAutoRed;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeFeeding;
import org.firstinspires.ftc.teamcode.Commands.LimelightSeesAT;
import org.firstinspires.ftc.teamcode.Commands.ShootAllThree;
import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Timer;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.List;

@TeleOp(group = "Test", name = "Test Auto")
public class TestLLAuto extends CommandOpMode {
    LLsubsystem limelight;
    ShooterSubsystem snap, crackle, pop;
    IntakeSubsystem intake;
    ChassisSubsystem chassis;
    ChassisDefaultFEILDCommand chassisDefault;
    IntakeDefaultCommand intakeDefaultCommand;
    ShooterDefaultCommand snapDefault, crackleDefault, popDefault;
    GamepadEx gamepad;
    TelemetryManager telemetryM;



    @Override
    public void initialize(){
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry.addLine("Init");
        telemetry.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = new LLsubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        //Chassis initialization
        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
        chassisDefault = new ChassisDefaultFEILDCommand(chassis, telemetryM, gamepad);
        //Shooter Initialization
        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
        crackle = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.CRACKLE);
        pop = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.POP);

        crackleDefault = new ShooterDefaultCommand(crackle);
        popDefault = new ShooterDefaultCommand(pop);
        snapDefault = new ShooterDefaultCommand(snap);
        //Intake Initialization
        intake = new IntakeSubsystem(hardwareMap, telemetryM);
        intakeDefaultCommand = new IntakeDefaultCommand(intake);
        //Trigger Initializations

        //Command based stuff
        register(snap, crackle, pop, chassis, intake);
        schedule(snapDefault, crackleDefault, popDefault, chassisDefault, intakeDefaultCommand);
        chassis.setDefaultCommand(chassisDefault);
        pop.setDefaultCommand(popDefault);
        crackle.setDefaultCommand(crackleDefault);
        snap.setDefaultCommand(snapDefault);
        intake.setDefaultCommand(intakeDefaultCommand);

        limelight.getLimelight().start();

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        limelight.getLimelight().updateRobotOrientation(chassis.yawPitchRollAngles().getYaw());
        if(gamepad1.start){
            chassis.resetPos();
        }
        //TODO: Hey Mr. Rubino, this is a way to shoot all three at once, just add it as a new command.
//        schedule(
//                new SequentialCommandGroup(new ChassisLookToAprilTagInAutoRed(chassis, limelight, telemetryM, 0),
//                        new ShootAllThree(snap, crackle, pop, telemetry))
//
//
//        );
        //TODO: this one is for intake, instead of the second timer put the drive command
//        schedule(
//                new SequentialCommandGroup(
//                        new ParallelDeadlineGroup(
//                                new Timer(RobotConstants.Teleop.DRIVE_FORWARD_AUTO),
//                                new IntakeCommand(intake, snap, crackle, pop),
//                                new IntakeFeeding(snap),
//                                new IntakeFeeding(pop)
//                        ),
//                        new ParallelCommandGroup(
//                                new Timer(RobotConstants.Teleop.DRIVE_FORWARD_AUTO),
//                                new IntakeCommand(intake, snap, crackle, pop)
//                        )
//
//
//                )
//        );
        Button pointAtAT = new GamepadButton(
                gamepad, GamepadKeys.Button.A
        ).whenPressed(
                new SequentialCommandGroup(
                        new ChassisLLAutoTurn(chassis, limelight, telemetry, -40),
                        new ParallelDeadlineGroup(
                              new LimelightSeesAT(limelight),
                                new ChassisAutoMoveUsingTime(chassis, telemetryM, 0.4 , 0.4, 0)
                        ),
                        new ChassisLLAutoDrive(chassis, limelight, telemetry, RobotConstants.Testing.CHASSIS_DRIVE_TO_SECOND_SHOT),
                        new ChassisLookToAprilTagInAutoRed(chassis, limelight, telemetryM, 0),
                        new ShootAllThree(snap, crackle, pop, telemetry),
                        new ChassisLLAutoDrive(chassis, limelight, telemetry, RobotConstants.Testing.CHASSIS_DRIVE_TO_FIRST_INTAKE),


                        new ChassisLLAutoTurn(chassis, limelight, telemetry, -90),
                        new ParallelDeadlineGroup(
                                new Timer(RobotConstants.Teleop.DRIVE_FORWARD_AUTO),
                                new IntakeCommand(intake, snap, crackle, pop),
                                new IntakeFeeding(snap),
                                new IntakeFeeding(pop),
                                new ChassisAutoMoveUsingTime(chassis, telemetryM, 0, -0.75, 0)
                        ),
                        new ParallelDeadlineGroup(
                                new ChassisLLAutoTurn(chassis, limelight, telemetry, 0),
                                new IntakeCommand(intake, snap, crackle, pop)
                        ),
                        new ParallelDeadlineGroup(
                                new LimelightSeesAT(limelight),
                                new ChassisAutoMoveUsingTime(chassis, telemetryM, -0.5, 0, 0),
                                new IntakeCommand(intake, snap, crackle, pop)
                        ),
                        new ChassisLLAutoDrive(chassis, limelight, telemetry, RobotConstants.Testing.CHASSIS_DRIVE_TO_SECOND_SHOT),
                        new ChassisLookToAprilTagInAutoRed(chassis, limelight, telemetryM, 0),
                        new ShootAllThree(snap, crackle, pop, telemetry),
                        new ChassisLLAutoDrive(chassis, limelight, telemetry, RobotConstants.Testing.CHASSIS_DRIVE_TO_FIRST_INTAKE),
                        new ChassisLLAutoDrive(chassis, limelight, telemetry, RobotConstants.Testing.CHASSIS_DRIVE_TO_SECOND_INTAKE),
                        new ParallelDeadlineGroup(
                            new Timer(RobotConstants.Teleop.DRIVE_FORWARD_AUTO),
                            new IntakeCommand(intake, snap, crackle, pop),
                            new IntakeFeeding(snap),
                            new IntakeFeeding(pop),
                            new ChassisAutoMoveUsingTime(chassis, telemetryM, 0, -0.75, 0)
                        ),
                        new ParallelDeadlineGroup(
                                new ChassisLLAutoTurn(chassis, limelight, telemetry, 0),
                                new IntakeCommand(intake, snap, crackle, pop)
                        ),
                        new ParallelDeadlineGroup(
                                new LimelightSeesAT(limelight),
                                new ChassisAutoMoveUsingTime(chassis, telemetryM, -0.5, 0, 0),
                                new IntakeCommand(intake, snap, crackle, pop)
                        ),
                        new ChassisLLAutoDrive(chassis, limelight, telemetry, RobotConstants.Testing.CHASSIS_DRIVE_TO_SECOND_SHOT),
                        new ChassisLookToAprilTagInAutoRed(chassis, limelight, telemetryM, 0),
                        new ShootAllThree(snap, crackle, pop, telemetry)
                        )

        );
        telemetry.addData("position", limelight.getLLResults().getBotpose_MT2());
        telemetry.addData("yaw", chassis.yawPitchRollAngles().getYaw());
        telemetryM.addData("yaw", chassis.yawPitchRollAngles().getYaw());

        telemetry.addData("Chassis Command", chassis.getCurrentCommand());
        telemetry.addData("ta", snap.getLLResult().getTa());
        telemetry.update();
        telemetryM.update();
    }
}

