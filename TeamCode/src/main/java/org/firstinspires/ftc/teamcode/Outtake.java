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
	int LOW_TICKS = -100;
	int MID_TICKS = 500;
	int HIGH_TICKS = 1400;

	public static double SERVO_ARMED = 0.8;
	public static double SERVO_DROP_LOW = 0.4;
	public static double SERVO_DROP_NORMAL = 0.4;
	public static double SERVO_LOADING = 1.0;

	public Outtake(OpMode opMode)
	{
		actionQueue = new ArrayList<>();

		motor = opMode.hardwareMap.get(DcMotor.class, "motorOuttake");
		servo = opMode.hardwareMap.get(Servo.class, "servoOuttake");

		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		servo.setPosition(SERVO_ARMED);
	}

	public void setLevel(Level level) {
		hasFinished = false;
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
		runAfter(delay, () -> {
			setLevel(level);
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
			hasFinished = true;
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
		return hasFinished;
	}

	void goToTicks(int targetTicks)
	{
		motor.setTargetPosition(targetTicks);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		motor.setPower(1);

	}
}
