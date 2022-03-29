package org.firstinspires.ftc.teamcode.specialized;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.CapstoneDetectPipeline;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RR.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.bt.actions.RunTrajectory;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunDelay;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunLinear;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelRace;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelWait;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeSetPower;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeWaitForElement;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntermediarySetRunning;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeDropFreight;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeSetLevel;
import org.firstinspires.ftc.teamcode.statics.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Autonomie Red Warehouse", group = "0prod")
public class AutoOpV3_Red_W extends AutonomousOpMode {

    Trajectory start_to_carousel, carousel_to_hub, warehouse_to_hub_c1, hub_to_warehouse_c1,
        warehouse_to_hub_c2, hub_to_duck, duck_to_hub, hub_to_park;

    CapstoneDetectPipeline capstoneDetection;
    WebcamName webcamName;
    OpenCvCamera camera;

    Outtake.Level preloadLevel = Outtake.Level.high;
    private Trajectory start_to_hub;


    @Override
    protected void precompileTrajectories() {
        PoseStorage.poseEstimate = new Pose2d(12.00, -63.00, Math.toRadians(90));

        // !!!! de inlocuit
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        start_to_carousel = drive.trajectoryBuilder(PoseStorage.poseEstimate, Math.toRadians(90))
            .splineTo(new Vector2d(-61.00, -60.00), Math.toRadians(180))
            .build();

        carousel_to_hub = drive.trajectoryBuilder(new Pose2d(-61, -60, Math.toRadians(180)), Math.toRadians(0))
            .splineTo(new Vector2d(-12.00, -42.00), Math.toRadians(90))
            .build();

        hub_to_warehouse_c1 = drive.trajectoryBuilder(new Pose2d(-12.00, -42.00, Math.toRadians(-90)), Math.toRadians(-90))
            .splineToSplineHeading(new Pose2d(17.00, -65.70, Math.toRadians(0)), Math.toRadians(0))
            .lineToLinearHeading(new Pose2d(43.00, -65.70, Math.toRadians(0)))
            .build();

        warehouse_to_hub_c1 = drive.trajectoryBuilder(new Pose2d(43.00, -65.70, Math.toRadians(0)), Math.toRadians(180))
            .lineToConstantHeading(new Vector2d(17.00, -65.70))
            .splineTo(new Vector2d(-12.00, -42.00), Math.toRadians(90))
            .build();

        warehouse_to_hub_c2 = drive.trajectoryBuilder(new Pose2d(43.00, -65.70, Math.toRadians(0)), Math.toRadians(190))
            .splineToConstantHeading(new Vector2d(23.00, -70.70), Math.toRadians(180))
            .splineTo(new Vector2d(-12.00, -45.00), Math.toRadians(90))
            .build();

        hub_to_duck = AssetsTrajectoryManager.load("hub_to_duck_v2");
//            drive.trajectoryBuilder(new Pose2d(-12.00, -43.00, Math.toRadians(-90)), Math.toRadians(-90))
//            .lineToConstantHeading(new Vector2d(-12.00, -50.00))
//            .lineToSplineHeading(new Pose2d(-12.00, -42.00, Math.toRadians(10)))
//            .splineToConstantHeading(new Vector2d(-6.00, -38.00), Math.toRadians(0))
//            .lineToSplineHeading(new Pose2d(16.00, -38.00, Math.toRadians(10)))
//            .build();

        duck_to_hub = drive.trajectoryBuilder(new Pose2d(16.00, -38.00, Math.toRadians(15)), Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(7.00, -33.00, Math.toRadians(-35)), Math.toRadians(180))
            .build();

        hub_to_park = AssetsTrajectoryManager.load("hub_to_park");
//            drive.trajectoryBuilder(new Pose2d(7.00, -24.00, Math.toRadians(-90)), Math.toRadians(0))
//            .splineToConstantHeading(new Vector2d(17.00, -43.79), Math.toRadians(-0))
//            .lineToConstantHeading(new Vector2d(80.00, -43.79))
//            .build();

        start_to_hub = AssetsTrajectoryManager.load("start_to_hub");
    }

    @Override
    protected void otherInit() {
        // CAMERA INITIALIZATION
        capstoneDetection = new CapstoneDetectPipeline();

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setPipeline(capstoneDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
//                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera couldn't init!!!" + "Error " + errorCode);
            }
        });


        telemetry.addLine("init complete");
        telemetry.update();

    }

    @Override
    protected void initLoop() {
        switch (capstoneDetection.capstoneSegment) {
            case 3:
                preloadLevel = Outtake.Level.high;
                telemetry.addLine("high");
                break;
            case 2:
                preloadLevel = Outtake.Level.mid;
                telemetry.addLine("mid");
                break;
            case 1:
                preloadLevel = Outtake.Level.low;
                telemetry.addLine("low");
                break;
            default:
                preloadLevel = Outtake.Level.high;
                telemetry.addLine("not detected but set to high");
        }
    }

    @Override
    protected Action getRoutine() {

        Action autoRoutine = new RunLinear(
            new RunDelay(14000),
            new RunParallelWait(
                new RunTrajectory(start_to_hub),
                new OuttakeSetLevel(preloadLevel)
            ),
            new OuttakeDropFreight(),
            new RunParallelWait(
                new OuttakeSetLevel(Outtake.Level.loading),
                new RunTrajectory(hub_to_warehouse_c1),
                new IntakeSetPower(-1),
                new IntermediarySetRunning(true),
                new RunParallelRace(
                    new IntakeWaitForElement(),
                    new RunDelay(3500)
                )
            ),
            new RunParallelWait(
                new RunTrajectory(warehouse_to_hub_c1),
                new RunLinear(
                    new IntakeSetPower(0),
                    new RunDelay(300),
                    new IntakeSetPower(-1),
                    new RunDelay(900),
                    new OuttakeSetLevel(Outtake.Level.high)
                )
            ),
            new OuttakeDropFreight(),

//            new RunParallelWait(
//                new OuttakeSetLevel(Outtake.Level.loading),
//                new RunTrajectory(hub_to_warehouse_c1),
//                new IntakeSetPower(-1),
//                new IntermediarySetRunning(true),
//                new RunParallelRace(
//                    new IntakeWaitForElement(),
//                    new RunDelay(3500)
//                )
//            ),
//            new RunParallelWait(
//                new RunTrajectory(warehouse_to_hub_c2),
//                new RunLinear(
//                    new IntakeSetPower(0),
//                    new RunDelay(300),
//                    new IntakeSetPower(-1),
//                    new RunDelay(900),
//                    new OuttakeSetLevel(Outtake.Level.high)
//                )
//            ),
//            new OuttakeDropFreight(),


            new IntermediarySetRunning(false),
            new IntakeSetPower(0),
            new RunParallelWait(
                new OuttakeSetLevel(Outtake.Level.loading),
                new RunTrajectory(hub_to_park)
            )
        );

        return autoRoutine;

    }
}
