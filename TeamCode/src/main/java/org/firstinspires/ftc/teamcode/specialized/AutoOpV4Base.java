package org.firstinspires.ftc.teamcode.specialized;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.CapstoneDetectPipeline;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.bt.ComposedAction;
import org.firstinspires.ftc.teamcode.bt.actions.RunTrajectory;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunBranch;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunDelay;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunEmpty;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunInfinite;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunInline;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunLinear;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelRace;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelWait;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeSetExtender;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeSetPower;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeWaitForElement;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntermediarySetRunning;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeDropFreight;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeSetLevel;
import org.firstinspires.ftc.teamcode.statics.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;

abstract public class AutoOpV4Base extends AutonomousOpMode {

	ArrayList<Trajectory> l_h_to_w = null;
	ArrayList<Trajectory> l_w_to_h = null;

	protected enum Side {
		RED,
		BLUE
	}

	protected enum StartLocation {
		WAREHOUSE,
		CAROUSEL
	}

	protected Side side = Side.RED;
	protected StartLocation startLocation = StartLocation.WAREHOUSE;

	Trajectory start_to_carousel, carousel_to_hub, warehouse_to_hub, hub_to_warehouse, hub_to_park, start_to_hub;

	protected String SIDE(String name) {
		return ((side == Side.RED) ? "red_" : "blue_") + name;
	}


	@Override
	protected void initStart() {

		if (side == Side.RED) {
			if (startLocation == StartLocation.WAREHOUSE)
				PoseStorage.poseEstimate = new Pose2d(12, -63.34, Math.toRadians(90));
			else
				PoseStorage.poseEstimate = new Pose2d(-36, -63.34, Math.toRadians(90));
		} else {
			if (startLocation == StartLocation.WAREHOUSE)
				PoseStorage.poseEstimate = new Pose2d(12, 63.34, Math.toRadians(-90));
			else
				PoseStorage.poseEstimate = new Pose2d(-36, 63.34, Math.toRadians(-90));
		}


		// RESET OUTTAKE TICKS

		outtake.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


		// CAMERA INITIALIZATION
		capstoneDetection = new CapstoneDetectPipeline();
//
//		webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//		camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//		camera.setPipeline(capstoneDetection);
//
//		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//			@Override
//			public void onOpened() {
//				camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
////                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
//			}
//
//			@Override
//			public void onError(int errorCode) {
//				telemetry.addLine("Camera couldn't init!!!" + "Error " + errorCode);
//			}
//		});


		telemetry.addLine("init complete");
		telemetry.update();
	}

	@Override
	protected void initTick() {
		preloadLevel = Outtake.Level.high;
		switch (capstoneDetection.capstoneSegment) {
			case 3:
				preloadLevel = Outtake.Level.high;
				telemetry.addLine("high");
				break;
			case 2:
				preloadLevel = Outtake.Level.mid_auto;

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


	protected class DoOneCycle extends ComposedAction {

		private final Trajectory h_to_w;
		private final Trajectory w_to_h;
		private boolean timeoutKills;


		public DoOneCycle(Trajectory h_to_w, Trajectory w_to_h, boolean timeoutKills) {

			this.h_to_w = h_to_w;
			this.w_to_h = w_to_h;
			this.timeoutKills = timeoutKills;
		}

		@SuppressLint("NewApi")
		@Override
		protected Action constructGroupAtStart(AutonomousOpMode context) {
//			return new RunLinear(
//					new RunParallelWait(
//							new OuttakeSetLevel(Outtake.Level.loading),
//							new RunTrajectory(h_to_w),
//							new IntakeSetPower(-1),
//							new IntermediarySetRunning(true),
//							new IntakeSetExtender(0.5),
//							new RunParallelRace(
//									new IntakeWaitForElement(),
//									new RunDelay(4500)
//							)
//					),
//					new IntakeSetPower(0),
//					new IntakeSetExtender(0),
////					!timeoutKills? new RunEmpty() :
////							new RunInline(ctx -> {
////								if(Arrays.stream(ctx.intake.touchSensors).anyMatch(it -> it.getState())) {
////									ctx.requestOpModeStop();
////								}
////							}),
//					new RunParallelWait(
//							new RunTrajectory(w_to_h),
//							new RunLinear(
//									new RunDelay(500),
//									new IntakeSetPower(-1),
//									new RunDelay(1300),
//									new OuttakeSetLevel(Outtake.Level.high),
//									new OuttakeDropFreight()
//							)
//					)
//			);
//		}
			return new RunLinear(
					new RunParallelWait(
							new RunTrajectory(h_to_w),
							new IntakeSetPower(-1),
							new IntermediarySetRunning(true),
							new OuttakeSetLevel(Outtake.Level.loading),
							new RunLinear(
									new RunParallelRace(
											new IntakeWaitForElement(),
											new RunDelay(4000)
									),
									new IntakeSetPower(1)
							)

					),
					new RunParallelWait(
							new RunTrajectory(w_to_h),
							new RunLinear(
									new RunDelay(700),
									new OuttakeSetLevel(Outtake.Level.high)
							)
					),
					new OuttakeDropFreight()
			);
		}
	}

	protected class DoNCycles extends ComposedAction {
		private final int noCycles;
		private Vector2d[] wPoints;
		private Vector2d[] hPoints;
		private Side side;
		private Trajectory transitionTrajectory;


		public DoNCycles(int noCycles, Vector2d[] wPoints, Vector2d[] hPoints, Side side) {
			this.noCycles = noCycles;
			this.wPoints = wPoints;
			this.hPoints = hPoints;
			this.side = side;

		}

		public DoNCycles(int noCycles, Vector2d[] wPoints, Vector2d[] hPoints, Side side, Trajectory transitionTrajectory) {
			this.noCycles = noCycles;
			this.wPoints = wPoints;
			this.hPoints = hPoints;
			this.side = side;
			this.transitionTrajectory = transitionTrajectory;
		}

		Trajectory get_h_to_w(AutonomousOpMode context, int cycleNo) {
			return context.drive.trajectoryBuilder(new Pose2d(-12.00 + hPoints[cycleNo].getX(), (-43.25 + hPoints[cycleNo].getY()) * (side == Side.RED ? 1 : -1), Math.toRadians(-90) * (side == Side.RED ? 1 : -1)), Math.toRadians(-90) * (side == Side.RED ? 1 : -1))
					.splineToSplineHeading(new Pose2d(17.00, (-65.70 + wPoints[cycleNo].getY()) * (side == Side.RED ? 1 : -1), Math.toRadians(0) * (side == Side.RED ? 1 : -1)), Math.toRadians(0) * (side == Side.RED ? 1 : -1))
					.lineToLinearHeading(new Pose2d(43.00 + wPoints[cycleNo].getX(), (-65.70 + wPoints[cycleNo].getY()) * (side == Side.RED ? 1 : -1), Math.toRadians(0) * (side == Side.RED ? 1 : -1)))
					.build();
		}

		Trajectory get_w_to_h(AutonomousOpMode context, int cycleNo) {
			return context.drive.trajectoryBuilder(new Pose2d(43.00 + wPoints[cycleNo].getX(), (-65.70 + wPoints[cycleNo].getY()) * (side == Side.RED ? 1 : -1), Math.toRadians(0) * (side == Side.RED ? 1 : -1)), Math.toRadians(180) * (side == Side.RED ? 1 : -1))
					.lineToConstantHeading(new Vector2d(17.00, (-65.70 + wPoints[cycleNo].getY()) * (side == Side.RED ? 1 : -1)))
					.splineTo(new Vector2d(-12.00 + hPoints[cycleNo + 1].getX(), (-43.25 + hPoints[cycleNo + 1].getY()) * (side == Side.RED ? 1 : -1)), Math.toRadians(90) * (side == Side.RED ? 1 : -1))
					.build();
		}

		@Override
		protected Action constructGroupAtStart(AutonomousOpMode context) {
			ArrayList<Action> cycles = new ArrayList<>();

			for (int i = 0; i < noCycles; i++) {
				cycles.add(
						new DoOneCycle(
								(i == 0 && transitionTrajectory != null) ?
										transitionTrajectory : get_h_to_w(context, i),
								get_w_to_h(context, i),
								i == noCycles - 1
						)
				);

			}

			return new RunLinear(cycles.toArray(new Action[0]));
		}
	}

}

