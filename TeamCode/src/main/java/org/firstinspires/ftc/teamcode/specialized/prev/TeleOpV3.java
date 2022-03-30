package org.firstinspires.ftc.teamcode.specialized.prev;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadEx;
import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeMechanism;

@Disabled
@TeleOp()
public class TeleOpV3 extends OpMode {

	// Gamepad 1
	// left stick, triggers - movement
	// Gamepad 2
	// A - toggle intake
	// B - reverse intake while pressed
	// left bumper - toggle carousel
	// dpad (down, left/right, up) - set outtake level (down, mid, up)
	// X - outtake discard freight and reset
	// Y - outtake reset to loading position !!NOT IMPLEMENTED

	boolean bumperLeftState;
	boolean aState;
	boolean bState;
	boolean xState;
	boolean dpadDownState;
	boolean dpadLeftState;
	boolean dpadRightState;
	boolean dpadUpState;


	Mecanum drive;

	DcMotor intakeMotor;
	DcMotor intermediaryMotor;
	boolean intakeState = false;

	DcMotor carouselMotor;
	boolean carouselState = false;

	Thread movementThread;

	OuttakeMechanism outtake;

	Servo capServo;

	@Override
	public void init() {
		drive = new Mecanum(this);
		outtake = new OuttakeMechanism(this);

		intakeMotor = hardwareMap.get(DcMotor.class, "motorIntake");
		intermediaryMotor = hardwareMap.get(DcMotor.class, "motorIntermediar");
		carouselMotor = hardwareMap.get(DcMotor.class, "motorCarousel");
//		capServo = hardwareMap.get(Servo.class, "capServo");
	}
	@Override
	public void loop() {
		drive.vectorMove(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.left_trigger - gamepad1.right_trigger, 0.6);

		if(gamepad2.left_bumper && !bumperLeftState)
		{
			carouselState = !carouselState;
		}
		bumperLeftState = gamepad2.left_bumper;

		if(gamepad2.a && !aState)
		{
			intakeState = !intakeState;
		}
		aState = gamepad2.a;

		carouselMotor.setPower(carouselState ? -0.27 : 0.0);
		intakeMotor.setPower(gamepad2.b ? 1.0 : intakeState ? -1.0 : 0.0);
		intermediaryMotor.setPower(gamepad2.b ? 1.0 : intakeState ? -1.0 : 0.0);

		if(gamepad2.dpad_down && !dpadDownState)
		{
			outtake.setStateAsync(OuttakeMechanism.State.LOW);
		}
		dpadDownState = gamepad2.dpad_down;

		if((gamepad2.dpad_left && !dpadLeftState) || (gamepad2.dpad_right && !dpadRightState))
		{
			outtake.setStateAsync(OuttakeMechanism.State.MID);
		}
		dpadLeftState = gamepad2.dpad_left;
		dpadRightState = gamepad2.dpad_right;

		if(gamepad2.dpad_up && !dpadUpState)
		{
			outtake.setStateAsync(OuttakeMechanism.State.HIGH);
		}
		dpadUpState = gamepad2.dpad_up;

		if(gamepad2.x && !xState)
		{
			outtake.setStateAsync(OuttakeMechanism.State.LOADING);
		}
		xState = gamepad2.x;

		if(gamepad2.right_bumper)
			capServo.setPosition(1);
		else
			capServo.setPosition(0);

	}
}
