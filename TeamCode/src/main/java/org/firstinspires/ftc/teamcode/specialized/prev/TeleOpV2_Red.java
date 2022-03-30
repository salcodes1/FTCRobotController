package org.firstinspires.ftc.teamcode.specialized.prev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadEx;
import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.Outtake;

@TeleOp(name = "TeleOp Red", group = "0")
public class TeleOpV2_Red extends OpMode {

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

    Servo capServo;
    Servo servoIntake;
    boolean servoIntakeState = false;

    DigitalChannel touchSensor;
    boolean touchSensorState = false;

    Outtake outtake;

    @Override
    public void init() {
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        drive = new Mecanum(this);
        outtake = new Outtake(this);

        intakeMotor = hardwareMap.get(DcMotor.class, "motorIntake");
		capServo = hardwareMap.get(Servo.class, "servoCapArm");
        intermediaryMotor = hardwareMap.get(DcMotor.class, "motorIntermediar");
        carouselMotor = hardwareMap.get(DcMotor.class, "motorCarousel");
        servoIntake = hardwareMap.get(Servo.class, "servoIntake");
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");

        servoIntake.setPosition(0);
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        capServo.setPosition(0.65);
    }

    @Override
    public void loop() {

        g1.update();
        g2.update();
        outtake.update();


        drive.vectorMove(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.left_trigger - gamepad1.right_trigger, 0.75);

        if (g2.getButtonDown("bumper_left")) carouselState = !carouselState;
        if (g2.getButtonDown("a")) intakeState = !intakeState;

        telemetry.update();

        carouselMotor.setPower(carouselState ? -0.33 : 0.0);
        intakeMotor.setPower(g2.getButton("b") ? 1.0 : intakeState ? -1.0 : 0.0);
        intermediaryMotor.setPower(g2.getButton("b") ? 1.0 : intakeState ? -1.0 : 0.0);

//	else if(gamepad2.right_trigger > 0) capServo.setPower(gamepad2.right_trigger * -0.4);
//	else capServo.setPower(0);

        if (g2.getButtonDown("dpad_down")) outtake.setLevel(Outtake.Level.low);
        if (g2.getButtonDown("dpad_left") || g2.getButtonDown("dpad_right"))
            outtake.setLevel(Outtake.Level.mid);
        if (g2.getButtonDown("dpad_up")) outtake.setLevel(Outtake.Level.high);
        if (g2.getButtonDown("x")) outtake.drop();

//        if(g1.getButtonDown("b"))
//            servoIntake.setPosition((servoIntakeState = !servoIntakeState)? 1 : 0);

        if(touchSensor.getState() && !touchSensorState) {
            touchSensorState = true;
            servoIntakeState = false;
            servoIntake.setPosition(0);
        } else if(!touchSensor.getState()) {
            touchSensorState = false;
        }

	}
}
