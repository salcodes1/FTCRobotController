package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.OuttakeMechanism;

import java.util.ArrayList;

@Config
public class Outtake {

	public DcMotor motor;
	public Servo servo;

	interface runAfterAction {
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

	boolean hasFinished = true;


	public enum Level {
		low,
		mid,
		high,
		loading
	};
	Level currentLevel;

	int LOADING_TICKS = 0;
	int LOW_TICKS = -150;
	int MID_TICKS = 500;
	int HIGH_TICKS = 1200;

	public static double SERVO_ARMED = 0.4;
	public static double SERVO_DROP_LOW = 0.05;
	public static double SERVO_DROP_NORMAL = 0.05;
	public static double SERVO_LOADING = 0.61;

	public Outtake(OpMode opMode)
	{
		actionQueue = new ArrayList<>();

		motor = opMode.hardwareMap.get(DcMotor.class, "elevationMotor");
		servo = opMode.hardwareMap.get(Servo.class, "containerServo");

		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		servo.setPosition(SERVO_ARMED);
	}

	public void setLevel(Level level) {
		currentLevel = level;
		switch (level) {

			case low:
				servo.setPosition(SERVO_ARMED);
				goToTicks(LOW_TICKS);
				break;
			case mid:
				servo.setPosition(SERVO_ARMED);
				goToTicks(MID_TICKS);
				break;
			case high:
				servo.setPosition(SERVO_ARMED);
				goToTicks(HIGH_TICKS);
				break;
			case loading:
				goToTicks(LOADING_TICKS);
				break;
		}
	}

	public void setLevelWithDelay(Level level, double delay)
	{
		hasFinished = false;
		runAfter(delay, () -> {
			setLevel(level);
			hasFinished = true;
		});
	}

	public void drop()
	{
		dropFor(700);
	}

	public void dropFor(int time)
	{
		servo.setPosition(currentLevel == Level.low ? SERVO_DROP_LOW :  SERVO_DROP_NORMAL);
		runAfter(time, () -> { servo.setPosition(SERVO_LOADING); });
		runAfter(time + 400, () -> { setLevel(Level.loading); });
	}

	void runAfter(double runAfter, runAfterAction action)
	{
		actionQueue.add(new runAfter(runAfter + System.currentTimeMillis(), action));
	}

	public void update()
	{

		if(!motor.isBusy())
		{
			motor.setPower(0);
		}

		for(int i = 0; i < actionQueue.size(); i++)
		{
			if(actionQueue.get(i).targetTime <= System.currentTimeMillis())
			{
				actionQueue.get(i).action.action();
				actionQueue.remove(i);
			}
		}
	}

	public boolean hasFinished()
	{
		return hasFinished && !motor.isBusy();
	}

	void goToTicks(int targetTicks)
	{
		motor.setTargetPosition(targetTicks);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		motor.setPower(0.75);

	}
}
