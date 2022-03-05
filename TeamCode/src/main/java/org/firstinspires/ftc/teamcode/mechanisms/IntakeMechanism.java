package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.InsideDetectPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class IntakeMechanism {

    // FFF IMPORTANT LA TUNING: FALSE POSITIVES MULT MAI OK DECAT FALSE NEGATIVES!
    public static double PIXELS_RATIO = 0.30f;
    public static double DETECTION_TRY_TIME_SEC = 100;
    public static double INTAKE_MOTOR_POWER = -1;
    public static double INTERMEDIARY_MOTOR_POWER = -1;
    // "motorBR" -- temporary hack
    public static String INTAKE_MOTOR_NAME = "intakeMotor";
    public static String INTERMEDIARY_MOTOR_NAME = "intermediaryMotor";
    public static String INTAKE_SERVO_NAME = "intakeServo";
    public static double INTAKE_SERVO_ENGAGED_POS = 1.0f;
    public static double INTAKE_SERVO_IDLE_POS = 0.0f;

    public static String TOUCH_SENSOR_1_NAME = "touchSensor1";
    public static String TOUCH_SENSOR_2_NAME = "touchSensor2";

    boolean hasFinished = false;

    Thread workThread;
    volatile boolean lastResult;


    DcMotor intakeMotor, intermediaryMotor;
    Servo intakeServo;

    DigitalChannel touchSensor1, touchSensor2;

    OpMode opMode;

    int elementsCount = 0;

    public IntakeMechanism(OpMode opMode) {

        this.opMode = opMode;

        intakeMotor = opMode.hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeServo = opMode.hardwareMap.get(Servo.class, INTAKE_SERVO_NAME);
        intakeServo.setPosition(0);

        intermediaryMotor = opMode.hardwareMap.get(DcMotor.class, INTERMEDIARY_MOTOR_NAME);

        touchSensor1 = opMode.hardwareMap.get(DigitalChannel.class, TOUCH_SENSOR_1_NAME);
        touchSensor2 = opMode.hardwareMap.get(DigitalChannel.class, TOUCH_SENSOR_2_NAME);

        touchSensor1.setMode(DigitalChannel.Mode.INPUT);
        touchSensor1.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean workHasFinished() {
        return workThread == null || (!workThread.isAlive()) || hasFinished;
    }

    public boolean getLastResult() { return lastResult; }

    public int getElementsCount() { return elementsCount; }

    public void toggleDirection()
    {
        if(intakeMotor.getDirection() == DcMotorSimple.Direction.REVERSE)
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        else
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void actionate()
    {
        hasFinished = false;
        lastResult = false;
        try {
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(INTAKE_MOTOR_POWER);
            intermediaryMotor.setPower(INTERMEDIARY_MOTOR_POWER);

            while(!Thread.interrupted()) {

                if(touchSensor1.getState() || touchSensor2.getState()) {
                    elementsCount++;
                    lastResult = true;
                    break;
                }
                Thread.yield();
            }
            hasFinished = true;
            intakeServo.setPosition(INTAKE_SERVO_IDLE_POS);
        }
        finally {
            hasFinished = true;
        }
    }

    public void haltMotors() {
        intakeMotor.setPower(0);
        intakeServo.setPosition(INTAKE_SERVO_IDLE_POS);
    }
    public void startWorkAsync(long timeout) {
        if(!workHasFinished()) return;

        if(timeout != -1) {
            Timer timeoutTimer = new Timer();
            timeoutTimer.schedule(new TimerTask() {
                @Override
                public void run() {
                    interruptWork();
                }
            }, timeout);
        }

        workThread = new Thread(() -> actionate());
        workThread.start();
    }

    public void interruptWork() {
        if(!workHasFinished())
            workThread.interrupt();
    }
}
