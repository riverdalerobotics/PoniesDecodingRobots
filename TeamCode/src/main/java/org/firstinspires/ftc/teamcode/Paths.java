//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathBuilder;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class Paths {
//    public static class GeneratedPaths {
//
//        public static PathBuilder builder = new PathBuilder();
//
//        public static PathChain Shoot = builder
//                .addPath(new BezierLine(new Pose(60.000, 9.348), new Pose(60.000, 84.000)))
//                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(138))
//                .build();
//
//        public static PathChain GoToTopIntake = builder
//                .addPath(new BezierLine(new Pose(60.000, 84.000), new Pose(43.000, 84.000)))
//                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))
//                .build();
//
//        public static PathChain Intaketop3 = builder
//                .addPath(new BezierLine(new Pose(43.000, 84.000), new Pose(17.000, 84.000)))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        public static PathChain ShootTwo = builder
//                .addPath(
//                        new BezierLine(new Pose(17.000, 84.000), new Pose(43.000, 100.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
//                .build();
//
//        public static PathChain GoToBottomIntake = builder
//                .addPath(
//                        new BezierLine(new Pose(43.000, 100.000), new Pose(43.000, 35.500))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))
//                .build();
//
//        public static PathChain IntakeBottom3 = builder
//                .addPath(new BezierLine(new Pose(43.000, 35.500), new Pose(17.000, 35.500)))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        public static PathChain ShootThree = builder
//                .addPath(
//                        new BezierCurve(
//                                new Pose(17.000, 35.500),
//                                new Pose(77.370, 28.840),
//                                new Pose(43.000, 100.000)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
//                .build();
//
//        public static PathChain GoToMidIntake = builder
//                .addPath(
//                        new BezierLine(new Pose(43.000, 100.000), new Pose(43.000, 60.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))
//                .build();
//
//        public static PathChain MidIntake = builder
//                .addPath(new BezierLine(new Pose(43.000, 60.000), new Pose(18.000, 60.000)))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        public static PathChain PrepForTeleop = builder
//                .addPath(
//                        new BezierCurve(
//                                new Pose(18.000, 60.000),
//                                new Pose(32.221, 58.475),
//                                new Pose(28.000, 70.000)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
//                .build();
//    }
//}
