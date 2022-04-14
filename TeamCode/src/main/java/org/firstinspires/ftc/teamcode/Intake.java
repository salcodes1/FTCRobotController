package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.FreightDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Config
public class Intake {

	public interface runAfterAction {
		void action();
	}

	class runAfter {
		runAfterAction action;
		double targetTime;

		runAfter(double targetTime, runAfterAction action)
		{
			this.action = action;
			this.targetTime = targetTime;
		}
	}

	ArrayList<runAfter> actionQueue;


	public DcMotor intakeMotor;
	public DcMotor intermediaryMotor;

	OpMode opMode;

	public Servo servoIntake;

	boolean isCurrentlyWorking = false;

	public FreightDetectionPipeline freightDetectionPipeline;
	public OpenCvWebcam freightCamera;
//	public Rev2mDistanceSensor distanceSensor;

	public Intake(OpMode opMode) {

		this.opMode = opMode;

		actionQueue = new ArrayList<>();

		intakeMotor = opMode.hardwareMap.get(DcMotor.class, "motorIntake");
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		intermediaryMotor = opMode.hardwareMap.get(DcMotor.class, "motorIntermediar");

		servoIntake = opMode.hardwareMap.get(Servo.class, "servoIntake");

		servoIntake.setPosition(0);

//		distanceSensor = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

		// CAMERA INITIALIZATION
		freightDetectionPipeline = new FreightDetectionPipeline(opMode.telemetry);

//		freightCamera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam"));
//		freightCamera.setPipeline(freightDetectionPipeline);
//
//		freightCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//			@Override
//			public void onOpened() {
//				freightCamera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
//			}
//
//			@Override
//			public void onError(int errorCode) {
//				opMode.telemetry.addLine("Freight camera couldn't init!!!" + "Error " + errorCode);
//			}
//		});


	}

	public void work()
	{
		intakeMotor.setPower(-1);
		intermediaryMotor.setPower(-1);
	}

	public void workFor(double time)
	{
		intakeMotor.setPower(-1);
		intermediaryMotor.setPower(-1);

		isCurrentlyWorking = true;
		runAfter(time, () -> {
			intakeMotor.setPower(0);
			intermediaryMotor.setPower(0);
			isCurrentlyWorking = false;
		});
	}

	public void ejectFor(double time)
	{
		intakeMotor.setPower(1);
		intermediaryMotor.setPower(1);
		isCurrentlyWorking = true;
		runAfter(time, () -> {
			intakeMotor.setPower(0);
			intermediaryMotor.setPower(0);
			isCurrentlyWorking = false;
		});
	}

	public void complexEject(double time, double intermediaryDelay)
	{
		intakeMotor.setPower(1);
		intermediaryMotor.setPower(-1);
		runAfter(intermediaryDelay, () -> {
			intermediaryMotor.setPower(1);
		});
		runAfter(time, () -> {
			intakeMotor.setPower(0);
			intermediaryMotor.setPower(0);
		});
	}

	public void ejectForWithDelay(double time, double delay)
	{
		runAfter(delay, () -> {
			ejectFor(time);
		});
	}

	public boolean isWorking()
	{
		return isCurrentlyWorking;
	}

	void runAfter(double runAfter, runAfterAction action)
	{
		actionQueue.add(new runAfter(runAfter + System.currentTimeMillis(), action));
	}

	public void stop()
	{
		intakeMotor.setPower(0);
		intermediaryMotor.setPower(0);
	}

	public void update()
	{

		for(int i = 0; i < actionQueue.size(); i++)
		{
			if(actionQueue.get(i).targetTime <= System.currentTimeMillis())
			{
				actionQueue.get(i).action.action();
				actionQueue.remove(i);
			}
		}
	}
}
