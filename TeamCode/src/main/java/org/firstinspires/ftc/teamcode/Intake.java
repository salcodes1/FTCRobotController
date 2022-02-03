package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.InsideDetectPipeline;
import org.firstinspires.ftc.teamcode.Outtake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

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


	DcMotor intakeMotor;
	DcMotor intermediaryMotor;

	OpMode opMode;

	boolean isCurrentlyWorking = false;

	public Intake(OpMode opMode) {

		this.opMode = opMode;

		actionQueue = new ArrayList<>();

		intakeMotor = opMode.hardwareMap.get(DcMotor.class, "intakeMotor");
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		intermediaryMotor = opMode.hardwareMap.get(DcMotor.class, "intermediaryMotor");

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
		intermediaryMotor.setPower(-1);
		isCurrentlyWorking = true;
		runAfter(time, () -> {
			intakeMotor.setPower(0);
			intermediaryMotor.setPower(0);
			isCurrentlyWorking = false;
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
