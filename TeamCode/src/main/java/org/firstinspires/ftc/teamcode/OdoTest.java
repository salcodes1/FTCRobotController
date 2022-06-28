package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OdoTest extends LinearOpMode {

	Servo sl, sc, sr;
	DcMotor l, c, r;

	@Override
	public void runOpMode() throws InterruptedException {

		sl = hardwareMap.get(Servo.class, "servoOdoLeft");
		sc = hardwareMap.get(Servo.class, "servoOdoCenter");
		sr = hardwareMap.get(Servo.class, "servoOdoRight");

		l = hardwareMap.get(DcMotor.class, "motorFL");
		c = hardwareMap.get(DcMotor.class, "motorFR");
		r = hardwareMap.get(DcMotor.class, "motorBL");

		waitForStart();

		sl.setPosition(0.0);
		sc.setPosition(1.0);
		sr.setPosition(1.0);

		while (opModeIsActive())
		{
			telemetry.addData("Left", l.getCurrentPosition());
			telemetry.addData("Center", c.getCurrentPosition());
			telemetry.addData("Right", r.getCurrentPosition());
			telemetry.update();
		}

	}
}
