package org.firstinspires.ftc.teamcode.CompCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ChassisAutoMoveUsingTime;
import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultFEILDCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisLLAutoDrive;
import org.firstinspires.ftc.teamcode.Commands.ChassisLLAutoDriveBLUE;
import org.firstinspires.ftc.teamcode.Commands.ChassisLLAutoTurn;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTagInAutoBlue;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTagInAutoRed;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeFeeding;
import org.firstinspires.ftc.teamcode.Commands.LimelightSeesAT;
import org.firstinspires.ftc.teamcode.Commands.RevToVeloUsingPID;
import org.firstinspires.ftc.teamcode.Commands.ShootAllThree;
import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.SpitCommand;
import org.firstinspires.ftc.teamcode.Commands.Timer;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.List;

@Autonomous(group = "Red Auto", name = " 9 Ball without gate")
public class Blue9Ball extends CommandOpMode {
    LLsubsystem limelight;
    ShooterSubsystem snap, crackle, pop;
    IntakeSubsystem intake;
    ChassisSubsystem chassis;
    ChassisDefaultFEILDCommand chassisDefault;
    IntakeDefaultCommand intakeDefaultCommand;
    ShooterDefaultCommand snapDefault, crackleDefault, popDefault;
    GamepadEx gamepad;
    TelemetryManager telemetryM;
    boolean run = true;



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
        chassis.resetPos();
        limelight.getLimelight().start();

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        if (run){schedule(new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new LimelightSeesAT(limelight),
                        new ChassisAutoMoveUsingTime(chassis, telemetryM, 0, 0.6 , 0)
                ),
                new ChassisLLAutoDriveBLUE(chassis, limelight, telemetry, RobotConstants.BlueAuto.BLUE_FIRST_INTAKE_POSE),
                new ChassisLookToAprilTagInAutoBlue(chassis, limelight, telemetryM, 0),
                new ShootAllThree(snap, crackle, pop, telemetry),
                new ChassisLLAutoTurn(chassis, limelight, telemetry, 90),
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
                        new Timer(RobotConstants.Teleop.DRIVE_FORWARD_AUTO),
                        new ChassisAutoMoveUsingTime(chassis, telemetryM, 0.4, 0, 0)
                ),
                new ChassisLLAutoTurn(chassis, limelight, telemetry, 30),
                new ParallelDeadlineGroup(
                        new LimelightSeesAT(limelight),
                        new ChassisAutoMoveUsingTime(chassis, telemetryM, 0.5, 0, 0),
                        new IntakeCommand(intake, snap, crackle, pop)
                ),
                new ChassisLLAutoDriveBLUE(chassis, limelight, telemetry, RobotConstants.BlueAuto.BLUE_SCORE_POSE),
                new ChassisLookToAprilTagInAutoBlue(chassis, limelight, telemetryM, 0),
                new ParallelDeadlineGroup(
                        new Timer(RobotConstants.Teleop.SHOOTER_TIMER),
                        new SpitCommand(intake),
                        new RevToVeloUsingPID(snap, telemetry),
                        new RevToVeloUsingPID(crackle, telemetry),
                        new RevToVeloUsingPID(pop, telemetry)
                ),
                new ShootAllThree(snap, crackle, pop, telemetry),
                new ChassisLLAutoDriveBLUE(chassis, limelight, telemetry, RobotConstants.BlueAuto.BLUE_SECOND_INTAKE_POSE),
                new ChassisLLAutoTurn(chassis, limelight, telemetry, 90),
                new ParallelDeadlineGroup(
                        new Timer(RobotConstants.Teleop.DRIVE_FORWARD_AUTO),
                        new IntakeCommand(intake, snap, crackle, pop),
                        new IntakeFeeding(snap),
                        new IntakeFeeding(pop),
                        new ChassisAutoMoveUsingTime(chassis, telemetryM, 0, -0.75, 0)
                ) ,
                new ParallelDeadlineGroup(
                        new Timer(RobotConstants.RedAuto.AUTO_TIME),
                        new ChassisAutoMoveUsingTime(chassis, telemetryM, 0, 0.5, 0),
                        new IntakeCommand(intake, snap, crackle, pop)
                ),
                new ParallelDeadlineGroup(
                        new ChassisLLAutoTurn(chassis, limelight, telemetry, 10),
                        new IntakeCommand(intake, snap, crackle, pop)
                ),
                new ParallelDeadlineGroup(
                        new LimelightSeesAT(limelight),
                        new ChassisAutoMoveUsingTime(chassis, telemetryM, 0.5, 0, 0),
                        new IntakeCommand(intake, snap, crackle, pop)
                ),
                new ChassisLLAutoDriveBLUE(chassis, limelight, telemetry, RobotConstants.BlueAuto.BLUE_SCORE_POSE),
                new ParallelDeadlineGroup(
                        new Timer(RobotConstants.Teleop.SHOOTER_TIMER),
                        new SpitCommand(intake),
                        new RevToVeloUsingPID(snap, telemetry),
                        new RevToVeloUsingPID(crackle, telemetry),
                        new RevToVeloUsingPID(pop, telemetry)
                ),
                new ChassisLookToAprilTagInAutoBlue(chassis, limelight, telemetryM, 0),
                new ShootAllThree(snap, crackle, pop, telemetry)
        ));
            run = false;
        }
        telemetry.addData("position", limelight.getLLResults().getBotpose_MT2());
        telemetry.addData("yaw", chassis.yawPitchRollAngles().getYaw());
        telemetryM.addData("yaw", chassis.yawPitchRollAngles().getYaw());

        telemetry.addData("Chassis Command", chassis.getCurrentCommand());
        telemetry.addData("ta", snap.getLLResult().getTa());
        telemetry.update();
        telemetryM.update();
    }
}
