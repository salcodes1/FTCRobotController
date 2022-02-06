package org.firstinspires.ftc.teamcode.specialized;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.OpenCV.CapstoneDetectPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.InsideDetectPipeline;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RR.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.statics.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
public class AutoOpV2 extends LinearOpMode {

	enum eAutoState {
		START_TO_CAROUSEL,
		CAROUSEL_RUNNING,
		CAROUSEL_TO_HUB,
		HUB_TO_WAREHOUSE,
		WAREHOUSE_TO_HUB,
		OUTTAKE_RUNNING,
		IDLE,
		HUB_TO_PARK
	}
	public static int CASE = 1;

	SampleMecanumDrive drive;

	Trajectory warehouse_to_hub_c1;
	Trajectory hub_to_warehouse_c1;

	Trajectory warehouse_to_hub_c2;
	Trajectory hub_to_warehouse_c2;

	Trajectory start_to_carousel;
	Trajectory carousel_to_hub;

	Trajectory hub_to_park;

	Intake intakeMechanism;
	Outtake outtakeMechanism;

	DcMotor carouselMotor;

	eAutoState state;

	CapstoneDetectPipeline capstoneDetection;
	WebcamName webcamName;
	OpenCvCamera camera;

	Outtake.Level preloadLevel = Outtake.Level.high;

	int cycles = 0;

	double startTime = 0;

	@Override
	public void runOpMode() throws InterruptedException {
		runInit();
		waitForStart();
		startTime = System.currentTimeMillis();

		state = eAutoState.START_TO_CAROUSEL;
		drive.setPoseEstimate(PoseStorage.poseEstimate);
		drive.followTrajectoryAsync(start_to_carousel);
		carouselMotor.setTargetPosition(-1350);
		carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		carouselMotor.setPower(-0.17);

		camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
			@Override
			public void onClose() {
				telemetry.addLine("camera successfully and gracefully closed thank god");
			}
		});

		while(opModeIsActive() && !isStopRequested())
		{
			run();
			outtakeMechanism.update();
			intakeMechanism.update();
			telemetry.addLine("time: " + (System.currentTimeMillis() - startTime) / 1000.0f);
			telemetry.update();
		}


	}

	private void run() {


		switch (state) {
			case IDLE: {
				// do Nothing
				break;
			}

			case START_TO_CAROUSEL: {
				if(!drive.isBusy()) {
					state = eAutoState.CAROUSEL_RUNNING;
					outtakeMechanism.setLevel(preloadLevel);
				}
				break;
			}

			case CAROUSEL_RUNNING: {
				if(!carouselMotor.isBusy()) {
					carouselMotor.setPower(0);
					drive.followTrajectoryAsync(carousel_to_hub);
					state = eAutoState.CAROUSEL_TO_HUB;
				}
				break;
			}

			case CAROUSEL_TO_HUB: {
				if(!drive.isBusy()) {
					outtakeMechanism.dropFor(200);
					drive.followTrajectoryAsync(hub_to_warehouse_c1);
					state = eAutoState.HUB_TO_WAREHOUSE;

					// already start to run the intake
					intakeMechanism.work();
				}
				break;
			}
			case HUB_TO_WAREHOUSE: {
				if(!drive.isBusy()) {
					outtakeMechanism.setLevelWithDelay(Outtake.Level.high, 3000);
//					intakeMechanism.complexEject(3000, 1500);

					if(cycles == 0) {
						drive.followTrajectoryAsync(warehouse_to_hub_c1);
					} else {
						drive.followTrajectoryAsync(warehouse_to_hub_c2);
					}

					state = eAutoState.WAREHOUSE_TO_HUB;
				}
				break;
			}
			case WAREHOUSE_TO_HUB: {
				if(!drive.isBusy()) {
					state = eAutoState.OUTTAKE_RUNNING;
				}
				break;
			}
			case OUTTAKE_RUNNING: {
				outtakeMechanism.dropFor(200);
				cycles++;

				if(cycles == 1) {
					drive.followTrajectoryAsync(hub_to_warehouse_c2);
					state = eAutoState.HUB_TO_WAREHOUSE;

					// already start to run the intake
					intakeMechanism.work();
				} else {
					drive.followTrajectoryAsync(hub_to_park);
					state = eAutoState.HUB_TO_PARK;
				}
				break;
			}
			case HUB_TO_PARK:
				break;

			default:
				break;
		}
		drive.update();
		PoseStorage.poseEstimate = drive.getPoseEstimate();
	}



	private void runInit() {
		drive = new SampleMecanumDrive(hardwareMap);
		intakeMechanism = new Intake(this);
		outtakeMechanism = new Outtake(this);

		carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");
		carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		PoseStorage.poseEstimate = new Pose2d(-36.00, -63.34, Math.toRadians(90));

		start_to_carousel 	= AssetsTrajectoryManager.load("start_to_carousel");
		carousel_to_hub 	= AssetsTrajectoryManager.load("carousel_to_hub");

		warehouse_to_hub_c1 = AssetsTrajectoryManager.load("warehouse_to_hub_c1");
		hub_to_warehouse_c1 = AssetsTrajectoryManager.load("hub_to_warehouse_c1");

		warehouse_to_hub_c2 = AssetsTrajectoryManager.load("warehouse_to_hub_c2");
		hub_to_warehouse_c2 = AssetsTrajectoryManager.load("hub_to_warehouse_c2");

		hub_to_park 		= AssetsTrajectoryManager.load("hub_to_park");

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

		while(!isStarted())
		{
			switch (capstoneDetection.capstoneSegment)
			{
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
			telemetry.update();
		}

	}
}
