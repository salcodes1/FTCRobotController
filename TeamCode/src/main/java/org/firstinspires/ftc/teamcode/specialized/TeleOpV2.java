package org.firstinspires.ftc.teamcode.specialized;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GamepadEx;
import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeMechanism;

@TeleOp()
public class TeleOpV2 extends OpMode {

	// Gamepad 1
	// left stick, triggers - movement
	// Gamepad 2
	// A - toggle intake
	// B - reverse intake while pressed
	// left bumper - toggle carousel
	// dpad (down, left/right, up) - set outtake level (down, mid, up)
	// X - outtake discard freight and reset
	// Y - outtake reset to loading position !!NOT IMPLEMENTED

	GamepadEx g1, g2;
	Mecanum drive;

	DcMotor intakeMotor;
	DcMotor intermediaryMotor;
	boolean intakeState = false;

	DcMotor carouselMotor;
	boolean carouselState = false;

	Outtake outtake;

	@Override
	public void init() {
		g1 = new GamepadEx(gamepad1);
		g2 = new GamepadEx(gamepad2);
		drive = new Mecanum(this);
		outtake = new Outtake(this);

		intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
		intermediaryMotor = hardwareMap.get(DcMotor.class, "intermediaryMotor");
		carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");
	}

	@Override
	public void loop() {

	g1.update();
	g2.update();
	outtake.update();


	drive.vectorMove(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.left_trigger - gamepad1.right_trigger, 0.6);

	if(g2.getButtonDown("bumper_left")) carouselState = !carouselState;
	if(g2.getButtonDown("a")) intakeState = !intakeState;

	carouselMotor.setPower(carouselState ? -0.27 : 0.0);
	intakeMotor.setPower(g2.getButton("b") ? 1.0 : intakeState ? -1.0 : 0.0);
	intermediaryMotor.setPower(g2.getButton("b") ? 1.0 : intakeState ? -1.0 : 0.0);

	if(g2.getButtonDown("dpad_down")) 									outtake.setLevel(Outtake.Level.low);
	if(g2.getButtonDown("dpad_left") || g2.getButtonDown("dpad_right")) outtake.setLevel(Outtake.Level.mid);
	if(g2.getButtonDown("dpad_up")) 									outtake.setLevel(Outtake.Level.high);
	if(g2.getButtonDown("x")) 											outtake.drop();


	}
}
