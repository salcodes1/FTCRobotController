package org.firstinspires.ftc.teamcode.specialized.prev;

import static org.firstinspires.ftc.teamcode.Outtake.SERVO_LOADING;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadEx;
import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.Outtake;

import java.util.Arrays;

@TeleOp(name = "TeleOp Blue", group = "0")
public class TeleOpV2_Blue extends OpMode {

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

    enum IntakeState {
        Automatic,
        Override,
        None
    }

    DcMotor intakeMotor;
    DcMotor intermediaryMotor;
    IntakeState intakeState = IntakeState.None;

    DcMotor carouselMotor;
    boolean carouselState = false;

    Servo servoCapArm;
    Servo servoCapClaw;

    double capPos = 0.65;

    Servo servoIntake;
    boolean servoIntakeState = false;

    DigitalChannel[] touchSensors;
    long[] touchSensorsLastTurnedOn;

    Outtake outtake;

    @Override
    public void init() {
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        drive = new Mecanum(this);
        outtake = new Outtake(this);

        intakeMotor = hardwareMap.get(DcMotor.class, "motorIntake");
        servoCapArm = hardwareMap.get(Servo.class, "servoCapArm");
        servoCapClaw = hardwareMap.get(Servo.class, "servoCapClaw");

        intermediaryMotor = hardwareMap.get(DcMotor.class, "motorIntermediar");
        carouselMotor = hardwareMap.get(DcMotor.class, "motorCarousel");
        servoIntake = hardwareMap.get(Servo.class, "servoIntake");

        outtake.servo.setPosition(SERVO_LOADING);

        touchSensors = new DigitalChannel[]{
                hardwareMap.get(DigitalChannel.class, "touchSensor1"),
                hardwareMap.get(DigitalChannel.class, "touchSensor2"),
                hardwareMap.get(DigitalChannel.class, "touchSensor3")
        };

        touchSensorsLastTurnedOn = new long[touchSensors.length];

        for(DigitalChannel channel : touchSensors) {
            channel.setMode(DigitalChannel.Mode.INPUT);
        }

        servoIntake.setPosition(0);
        servoCapArm.setPosition(capPos);

        servoCapClaw.setPosition(1);

    }

    @SuppressLint("NewApi")
    @Override
    public void loop() {

        g1.update();
        g2.update();
        outtake.update();


        if(Math.abs(gamepad2.right_stick_y) > 0.4) {
            outtake.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Log.d("FF", Double.toString(gamepad2.left_stick_y * 0.5));
            outtake.motor.setPower(gamepad2.right_stick_y * 0.5);
        }
        if(g2.getButtonDown("joystick_right")) outtake.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.vectorMove(-gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.left_trigger - gamepad1.right_trigger,
                (gamepad1.right_bumper? 0.3 : 0.75));

        if (g2.getButtonDown("bumper_left")) carouselState = !carouselState;

        telemetry.update();

        carouselMotor.setPower(carouselState ? 0.4 : 0.0);
        intakeMotor.setPower(g2.getButton("b") ? 1.0 : intakeState != IntakeState.None? -1.0 : 0.0);
        intermediaryMotor.setPower(g2.getButton("b") ? 1.0 : intakeState != IntakeState.None ? -1.0 : 0.0);


        if(gamepad2.left_trigger > 0.1 && capPos > 0.15) {
            capPos -= 0.002;
        }
        if(gamepad2.right_trigger > 0.1 && capPos < 0.65) {
            capPos += 0.002;
        }

        if (g2.getButtonDown("y")) {
            if(capPos < (0.15 + 0.65) / 2) capPos = 0.65;
            else capPos = 0.15;
        }

        if(g2.getButtonDown("bumper_right")) {
            servoCapClaw.setPosition(servoCapClaw.getPosition() > 0? 0 : 1);
        }

        servoCapArm.setPosition(capPos);

        if (g2.getButtonDown("dpad_down")) outtake.setLevel(Outtake.Level.low);
        if (g2.getButtonDown("dpad_left") || g2.getButtonDown("dpad_right"))
            outtake.setLevel(Outtake.Level.mid);
        if (g2.getButtonDown("dpad_up")) outtake.setLevel(Outtake.Level.high);
        if (g2.getButtonDown("x")) outtake.drop();

        if(g1.getButtonDown("b"))
            servoIntake.setPosition((servoIntakeState = !servoIntakeState)? 1 : 0);



        if (g2.getButtonDown("a")) intakeState = intakeState == IntakeState.Automatic? IntakeState.None : IntakeState.Automatic;
        else {
            if(intakeState == IntakeState.Automatic && Arrays.stream(touchSensors).anyMatch(DigitalChannel::getState)) {
                intakeState = IntakeState.None;
            }

            if(intakeState != IntakeState.Automatic) {
                if(gamepad2.left_stick_x > 0.2 || gamepad2.left_stick_y > 0.2) {
                    intakeState = IntakeState.Override;
                } else {
                    intakeState = IntakeState.None;
                }
            }

        }




    }

}
