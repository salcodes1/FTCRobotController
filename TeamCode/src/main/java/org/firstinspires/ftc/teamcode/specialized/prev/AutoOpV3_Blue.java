package org.firstinspires.ftc.teamcode.specialized.prev;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.CapstoneDetectPipeline;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RR.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.bt.actions.RunCarousel;
import org.firstinspires.ftc.teamcode.bt.actions.RunTrajectory;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunLinear;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelWait;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeDropFreight;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeSetLevel;
import org.firstinspires.ftc.teamcode.statics.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Autonomie Blue", group = "0prod")
public class AutoOpV3_Blue extends AutonomousOpMode {

    Trajectory start_to_carousel, carousel_to_hub, warehouse_to_hub_c1, hub_to_warehouse_c1,
        warehouse_to_hub_c2, hub_to_duck, duck_to_hub, hub_to_park;

    CapstoneDetectPipeline capstoneDetection;
    WebcamName webcamName;
    OpenCvCamera camera;

    Outtake.Level preloadLevel = Outtake.Level.high;
    private Trajectory hto;


    @Override
    protected void precompileTrajectories() {
        PoseStorage.poseEstimate = new Pose2d(-36.00, 63.34, Math.toRadians(-90));

        // !!!! de inlocuit
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        start_to_carousel = AssetsTrajectoryManager.load("blue-start_to_carousel");

        carousel_to_hub = AssetsTrajectoryManager.load("blue-carousel_to_hub");

        hub_to_warehouse_c1 = AssetsTrajectoryManager.load("blue-hub_to_warehouse_c1");

        warehouse_to_hub_c1 = AssetsTrajectoryManager.load("blue-warehouse_to_hub_c1");

//        warehouse_to_hub_c2 = AssetsTrajectoryManager.load("blue-");

        hub_to_duck = AssetsTrajectoryManager.load("blue-hub_to_duck_v2");

        duck_to_hub = AssetsTrajectoryManager.load("blue-duck_to_hub");

        hub_to_park = AssetsTrajectoryManager.load("blue-hub_to_park");

        hto = AssetsTrajectoryManager.load("blue-hub_to_sto");
    }

    @Override
    protected void initStart() {
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
    protected void initTick() {
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
            new RunParallelWait(
                new RunTrajectory(start_to_carousel),
                new RunCarousel(2125, 0.25)
            ),
            new RunParallelWait(
                new RunTrajectory(carousel_to_hub),
                new OuttakeSetLevel(preloadLevel)
            ),
            new OuttakeDropFreight(),
            new RunParallelWait(
                new OuttakeSetLevel(Outtake.Level.loading),
                new RunTrajectory(hub_to_park)
            )
        );

        return autoRoutine;

    }
}
