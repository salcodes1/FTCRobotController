package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

    Thread workThread;
    volatile boolean lastResult;


    DcMotor intakeMotor, intermediaryMotor;
    Servo intakeServo;


    WebcamName webcamName;
    OpenCvCamera camera;
    InsideDetectPipeline insideDetectPipeline;

    OpMode opMode;

    int elementsCount = 0;

    public IntakeMechanism(OpMode opMode) {
        insideDetectPipeline = new InsideDetectPipeline(opMode.telemetry);


        this.opMode = opMode;


        webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        camera.setPipeline(insideDetectPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addLine("Camera couldn't init!!!" + "Error " + errorCode);
            }
        });

        intakeMotor = opMode.hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeServo = opMode.hardwareMap.get(Servo.class, INTAKE_SERVO_NAME);
        intakeServo.setPosition(0);

        intermediaryMotor = opMode.hardwareMap.get(DcMotor.class, INTERMEDIARY_MOTOR_NAME);

    }

    public boolean workHasFinished() {
        return workThread == null || (!workThread.isAlive());
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

        lastResult = false;
        try {
            // (positively?) insane method for changing a variable in inner and reading it in outer

            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(INTAKE_MOTOR_POWER);
            intermediaryMotor.setPower(INTERMEDIARY_MOTOR_POWER);

            while(!Thread.interrupted()) {
//                camera.openCameraDevice();

                if(insideDetectPipeline.pieceInside()) {
                    elementsCount++;
                    lastResult = true;
                    break;
                }
                Thread.yield();
            }

            intakeServo.setPosition(INTAKE_SERVO_IDLE_POS);
            Thread.sleep(200);
            // inverting the direction so it doesnt take any other elems
            intakeMotor.setPower(-INTAKE_MOTOR_POWER);
            Thread.sleep(1500);
            intakeMotor.setPower(0);
            Thread.sleep(1500);
        } catch (InterruptedException ignored) {
        } finally {
            intakeMotor.setPower(0);
            intakeServo.setPosition(INTAKE_SERVO_IDLE_POS);
            intermediaryMotor.setPower(0);
        }
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

//        workThread = new Thread(() -> actionate());
        workThread.start();
    }

    public void interruptWork() {
        if(!workHasFinished())
            workThread.interrupt();
    }
}
