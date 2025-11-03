package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(15);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName(RobotConstants.Hardware.OTOS_SENSOR)
            .offset(new SparkFunOTOS.Pose2D(2, 0, 90))
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .OTOSLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(RobotConstants.Hardware.driveConstants)
                .build();
    }

}
