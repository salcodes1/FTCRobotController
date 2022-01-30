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
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        insideDetectPipeline = new InsideDetectPipeline(opMode.telemetry);


        this.opMode = opMode;


        webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
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

    public void DEB_ToggleIntermediary() {
        if(intermediaryMotor.getPower() > 0)
            intermediaryMotor.setPower(0);
        else
            intermediaryMotor.setPower(INTERMEDIARY_MOTOR_POWER);
    }

    public void DEB_ToggleIntake() {
        if(intakeMotor.getPower() < 0)
            intakeMotor.setPower(0);
        else
            intakeMotor.setPower(-1);
    }

    public void DEB_ToggleServo() {
        if(intakeServo.getPosition() == 1)
            intakeServo.setPosition(0);
        else
            intakeServo.setPosition(1);
    }

    public boolean workHasFinished() {
        return workThread == null || (!workThread.isAlive());
    }

    public boolean getLastResult() { return lastResult; }

    public int getElementsCount() { return elementsCount; }

    public void toggleDirection()
    {
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    boolean actionate()
    {
        try {
            // (positively?) insane method for changing a variable in inner and reading it in outer

            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(INTAKE_MOTOR_POWER);
            intermediaryMotor.setPower(INTERMEDIARY_MOTOR_POWER);
//            intakeServo.setPosition(INTAKE_SERVO_ENGAGED_POS);

            while(!Thread.interrupted()) {
                camera.openCameraDevice();

                if(insideDetectPipeline.pieceInside()) {
                    elementsCount++;
                    break;
                }
                Thread.yield();
            }

            intakeServo.setPosition(INTAKE_SERVO_IDLE_POS);
            Thread.sleep(1000);
            // inverting the direction so it doesnt take any other elems
            intakeMotor.setPower(-INTAKE_MOTOR_POWER);
            Thread.sleep(1500);
            intakeMotor.setPower(0);
            Thread.sleep(1500);
        } catch (InterruptedException ignored) {
            return false;
        } finally {
            intakeMotor.setPower(0);
            intakeServo.setPosition(INTAKE_SERVO_IDLE_POS);
            intermediaryMotor.setPower(0);
        }
        return !Thread.interrupted();
    }

    public void startWorkAsync() {
        if(!workHasFinished()) return;

        workThread = new Thread(() -> {
            lastResult = actionate();
        });
        workThread.start();
    }

    public void interruptWork() {
        if(!workHasFinished())
            workThread.interrupt();
    }
}
