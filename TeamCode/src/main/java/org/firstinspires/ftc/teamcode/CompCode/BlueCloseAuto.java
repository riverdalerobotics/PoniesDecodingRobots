package org.firstinspires.ftc.teamcode.CompCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ChassisAutoMoveUsingTime;
import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultAuto;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTagInAutoBlue;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTagInAutoClose;
import org.firstinspires.ftc.teamcode.Commands.ShootSequence;
import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Timer;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@Autonomous(group = "Comp", name = "1/3 Blue Auto CLose")
public class BlueCloseAuto extends CommandOpMode {
    ChassisSubsystem chassis;
    ShooterSubsystem snap;
    ChassisDefaultAuto chassisDefault;
    ShooterDefaultCommand snapDefault;
    TelemetryManager telemetryM;
    LLsubsystem limelight;
    boolean run = true;
    @Override
    public void initialize() {
        limelight = new LLsubsystem(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
        snapDefault = new ShooterDefaultCommand(snap);
        chassisDefault = new ChassisDefaultAuto(chassis, telemetryM);
        register(chassis, snap);
        schedule(snapDefault, chassisDefault);
        snap.setDefaultCommand(snapDefault);
        chassis.setDefaultCommand(chassisDefault);
        chassis.initBlue();
        chassis.resetPos();


    }

    @Override
    public void run() {
        super.run();
        CommandScheduler.getInstance().run();
        if(run){
            schedule(
                    new SequentialCommandGroup(
                            new ChassisLookToAprilTagInAutoClose(chassis, limelight, telemetryM, 0),
                            new ShootSequence(snap),
                            new ShootSequence(snap),
                            new ShootSequence(snap),
                            new ParallelDeadlineGroup(
                                    new Timer(RobotConstants.Teleop.DRIVE_FORWARD_CLOSE_AUTO),
                                    new ChassisAutoMoveUsingTime(chassis, telemetryM, 0.1,0,0)
                            )
                    ));
            run = false;
        }

    }
}
