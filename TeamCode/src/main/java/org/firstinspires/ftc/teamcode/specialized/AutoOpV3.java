package org.firstinspires.ftc.teamcode.specialized;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RR.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutoRobot;
import org.firstinspires.ftc.teamcode.bt.actions.RunCarousel;
import org.firstinspires.ftc.teamcode.bt.actions.RunTrajectory;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunDelay;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunLinear;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelRace;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelWait;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunRepeated;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeSetRunning;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeDropFreight;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeSetLevel;
import org.firstinspires.ftc.teamcode.statics.PoseStorage;
import org.opencv.core.Mat;

@Autonomous
public class AutoOpV3 extends LinearOpMode {

    volatile AutoRobot autoRobot;

    Trajectory start_to_carousel, carousel_to_hub, warehouse_to_hub_c1, hub_to_warehouse_c1,
        warehouse_to_hub_c2, hub_to_duck, duck_to_hub, hub_to_park;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        autoRobot = new AutoRobot(this);

        PoseStorage.poseEstimate = new Pose2d(-36.00, -63.34, Math.toRadians(90));

        precompileTrajectories();

        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();

        Outtake.Level l = Outtake.Level.high; // TODO!!!!!!

//        Func<Action> DoCycle = () -> new RunLinear(
//            new RunParallelWait(
//                new IntakeSetRunning(true),
//                new RunTrajectory(hub_to_warehouse_c1)
//            ),
//            new RunDelay(1000),
//            new RunParallelWait(
//                new RunTrajectory(warehouse_to_hub_c1),
//                new OuttakeSetLevel(Outtake.Level.high)
//            ),
//            new RunParallelWait(
//                new OuttakeDropFreight(),
//                new RunDelay(400)
//            ),
//            new OuttakeSetLevel(Outtake.Level.loading)
//        );

//        autoRobot.start(
//            new RunLinear(
//                new RunTrajectory(start_to_carousel),
//                new RunCarousel(-1350, -0.37),
//                new RunParallelWait(
//                    new RunTrajectory(carousel_to_hub),
//                    new OuttakeSetLevel(l)
//                ),
//                new RunParallelWait(
//                    new OuttakeDropFreight(),
//                    new RunDelay(400)
//                ),
//                // 2 cicluri
//                new RunRepeated(2, DoCycle.value())
//            )
//        );
        Action routine = new RunLinear(
            new RunDelay(3000),
            new RunParallelWait(
                new RunDelay(8000),
                new RunDelay(2000)
            ),
            new RunParallelRace(
                new RunDelay(2000),
                new RunDelay(5000)
            ),
            new RunDelay(3000),
            new RunParallelWait(
                new RunDelay(8000),
                new RunDelay(2000)
            ),
            new RunParallelRace(
                new RunDelay(2000),
                new RunDelay(5000)
            ),
            new RunDelay(3000),
            new RunParallelWait(
                new RunDelay(8000),
                new RunDelay(2000)
            ),
            new RunParallelRace(
                new RunDelay(2000),
                new RunDelay(5000)
            )
        );


        autoRobot.start(routine);
    }

    private void precompileTrajectories() {
        PoseStorage.poseEstimate = new Pose2d(-36.00, -63.34, Math.toRadians(90));

        // !!!! de inlocuit
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        start_to_carousel = drive.trajectoryBuilder(PoseStorage.poseEstimate, Math.toRadians(90))
            .splineTo(new Vector2d(-61.00, -60.00), Math.toRadians(180))
            .build();

        carousel_to_hub = drive.trajectoryBuilder(new Pose2d(-61, -60, Math.toRadians(180)), Math.toRadians(0))
            .splineTo(new Vector2d(-12.00, -41.00), Math.toRadians(90))
            .build();

        warehouse_to_hub_c1 = drive.trajectoryBuilder(new Pose2d(43.00, -65.70, Math.toRadians(0)), Math.toRadians(180))
            .lineToConstantHeading(new Vector2d(17.00, -65.70))
            .splineToConstantHeading(new Vector2d(-12.00, -42.00), Math.toRadians(90))
            .build();

        warehouse_to_hub_c2 = drive.trajectoryBuilder(new Pose2d(43.00, -65.70, Math.toRadians(0)), Math.toRadians(190))
            .splineToConstantHeading(new Vector2d(23.00, -70.70), Math.toRadians(180))
            .splineToConstantHeading(new Vector2d(-12.00, -43.00), Math.toRadians(90))
            .build();

        hub_to_duck = drive.trajectoryBuilder(new Pose2d(-12.00, -43.00, Math.toRadians(-90)), Math.toRadians(-90))
            .lineToConstantHeading(new Vector2d(-12.00, -50.00))
            .lineToSplineHeading(new Pose2d(-12.00, -42.00, Math.toRadians(10)))
            .splineToConstantHeading(new Vector2d(-6.00, -38.00), Math.toRadians(0))
            .lineToSplineHeading(new Pose2d(16.00, -38.00, Math.toRadians(10)))
            .build();

        duck_to_hub = drive.trajectoryBuilder(new Pose2d(16.00, -38.00, Math.toRadians(15)), Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(7.00, -33.00, Math.toRadians(-35)), Math.toRadians(180))
            .build();

        hub_to_park = drive.trajectoryBuilder(new Pose2d(7.00, -24.00, Math.toRadians(-90)), Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(17.00, -43.79), Math.toRadians(-0))
            .lineToConstantHeading(new Vector2d(80.00, -43.79))
            .build();
    }
}
