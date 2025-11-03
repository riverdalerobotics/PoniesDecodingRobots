package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.Shoot;
import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(group = "Test Auto", name = "Blue test auto")
public class BlueTestAuto extends CommandOpMode{
    ChassisSubsystem chassis;
//    ShooterSubsystem snap, crackle, pop;
//    IntakeSubsystem intake;
    Follower follower;
    TelemetryManager telemetryM;
    PathBuilder builder;
    PathChain Shoot, GoToTopIntake, Intaketop3, ShootTwo, GoToBottomIntake, IntakeBottom3,
            ShootThree, GoToMidIntake, MidIntake, PrepForTeleop;
//    ShooterDefaultCommand snapDefault, crackleDefault, popDefault;
    GamepadEx driver;
    Shoot shootCommand;
//    LLResult result;
    char[] gpp = {'g', 'p', 'p'};
    char[] ppg = {'p', 'p', 'g'};
    char[] pgp = {'p', 'g', 'p'};
    char[] motif = ppg;
    public void buildPaths(){
        Shoot = builder
                .addPath(new BezierLine(new Pose(60.000, 9.348), new Pose(60.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(138))
                .build();

        GoToTopIntake = builder
                .addPath(new BezierLine(new Pose(60.000, 84.000), new Pose(43.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))
                .build();

        Intaketop3 = builder
                .addPath(new BezierLine(new Pose(43.000, 84.000), new Pose(17.000, 84.000)))
                .setTangentHeadingInterpolation()
                .build();

        ShootTwo = builder
                .addPath(
                        new BezierLine(new Pose(17.000, 84.000), new Pose(43.000, 100.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                .build();

        GoToBottomIntake = builder
                .addPath(
                        new BezierLine(new Pose(43.000, 100.000), new Pose(43.000, 35.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))
                .build();

        IntakeBottom3 = builder
                .addPath(new BezierLine(new Pose(43.000, 35.500), new Pose(17.000, 35.500)))
                .setTangentHeadingInterpolation()
                .build();

        ShootThree = builder
                .addPath(
                        new BezierCurve(
                                new Pose(17.000, 35.500),
                                new Pose(77.370, 28.840),
                                new Pose(43.000, 100.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                .build();

        GoToMidIntake = builder
                .addPath(
                        new BezierLine(new Pose(43.000, 100.000), new Pose(43.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))
                .build();

        MidIntake = builder
                .addPath(new BezierLine(new Pose(43.000, 60.000), new Pose(18.000, 60.000)))
                .setTangentHeadingInterpolation()
                .build();

        PrepForTeleop = builder
                .addPath(
                        new BezierCurve(
                                new Pose(18.000, 60.000),
                                new Pose(32.221, 58.475),
                                new Pose(28.000, 70.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }
    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        builder = new PathBuilder(follower);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
//        intake = new IntakeSubsystem(hardwareMap, telemetryM);
//        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
//        pop = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.POP);
//        crackle  = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.CRACKLE);
//        snapDefault = new ShooterDefaultCommand(snap);
//        crackleDefault = new ShooterDefaultCommand(crackle);
//        popDefault = new ShooterDefaultCommand(pop);
        driver = new GamepadEx(gamepad1);
//        shootCommand = new Shoot(snap, crackle, pop, motif, true);
        buildPaths();
    }

    @Override
    public void run() {
        super.run();
        if(chassis.id == 21){
            motif = gpp;
        } else if (chassis.id == 22) {
            motif = pgp;
        } else if (chassis.id == 23) {
            motif = ppg;
        }

        schedule(new SequentialCommandGroup(
                new FollowPath(Shoot, chassis, telemetryM),
                shootCommand,
                new FollowPath(GoToTopIntake, chassis, telemetryM),
//                new ParallelRaceGroup(new FollowPath(Intaketop3, chassis, telemetryM), new IntakeCommand(intake)),
                new FollowPath(ShootTwo, chassis, telemetryM),
                shootCommand,
                new FollowPath(GoToBottomIntake, chassis, telemetryM),
//                new ParallelRaceGroup(new FollowPath(IntakeBottom3, chassis, telemetryM), new IntakeCommand(intake)),
                new FollowPath(ShootThree, chassis, telemetryM),
                shootCommand,
                new FollowPath(GoToMidIntake, chassis, telemetryM),
//                new ParallelRaceGroup(new FollowPath(MidIntake, chassis, telemetryM), new IntakeCommand(intake)),
                new FollowPath(PrepForTeleop, chassis, telemetryM)
                ));
    }
}
