package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Dictionary;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class OuttakeMechanism {
    public enum State {
        LOADING,
        LOW,
        MID,
        HIGH
    }

    public static int LOADING_TICKS = 0;
    public static int LOW_TICKS = -150;
    public static int MID_TICKS = 500;
    public static int HIGH_TICKS = 1200;

    volatile AtomicReference<State> currentState = new AtomicReference<>();

    Thread workThread;

    OpMode opMode;

    DcMotor elevationMotor;
    Servo containerServo;

    State PrevState;


    public OuttakeMechanism(OpMode opMode) {
        this.opMode = opMode;

        elevationMotor = opMode.hardwareMap.get(DcMotor.class, "elevationMotor");
        containerServo = opMode.hardwareMap.get(Servo.class, "containerServo");

        elevationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        containerServo.setPosition(0.6);

    }

    void setState(State state) {
        if(currentState.get() == state) return;

        currentState.set(state);
        switch (currentState.get()) {
            case LOADING: {
                try {
                    containerServo.setPosition(PrevState == PrevState.LOW ? 0.2 :  0.3);
                    Thread.sleep(700);
                    containerServo.setPosition(1);
                    Thread.sleep(400);
                }
                catch (InterruptedException e) {
                }
                goToTicks(LOADING_TICKS);
                break;
            }
            case LOW: {
                PrevState = State.LOW;
                containerServo.setPosition(0.6);
                goToTicks(LOW_TICKS);
                break;
            }
            case MID: {
                PrevState = State.MID;
                containerServo.setPosition(0.6);
                goToTicks(MID_TICKS);
                break;
            }
            case HIGH: {
                PrevState = State.HIGH;
                containerServo.setPosition(0.6);
                goToTicks(HIGH_TICKS);
                break;
            }
        }
    }

    public boolean workHasFinished() {
        return workThread == null || (!workThread.isAlive());
    }

    public void setStateAsync(State state) {
        // potentially unsafe; time will tell..

        if(state == currentState.get()) return;

        if(workThread != null && workThread.isAlive())
            workThread.interrupt();

        workThread = new Thread(() -> {
            setState(state);
        });
        workThread.start();
    }

    void goToTicks(int targetTicks)
    {
        elevationMotor.setTargetPosition(targetTicks);
        elevationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if(elevationMotor.getCurrentPosition() < elevationMotor.getTargetPosition()) {
//            elevationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        } else {
//            elevationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        }

        elevationMotor.setPower(0.75);
        while(elevationMotor.isBusy() && !Thread.interrupted()) {
            Thread.yield();
        }
        elevationMotor.setPower(0);
    }

}
