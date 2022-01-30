package org.firstinspires.ftc.teamcode.specialized;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.RR.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeMechanism;

import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "TeleOp Version 0.1")
public class TeleOpV1 extends OpMode {

//    SampleMecanumDrive drive;
    Mecanum drive;
    AtomicReference<WorkState> state;
    IntakeMechanism intakeMechanism;
    OuttakeMechanism outtakeMechanism;

    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

//    GamepadEx bGamepad = new GamepadEx(gamepad1);


    @Override
    public void init() {
//        drive = new SampleMecanumDrive(hardwareMap);
        drive = new Mecanum(this);
        state = new AtomicReference<>(WorkState.SELECT_LEVEL_AND_CONFIRM);
        intakeMechanism = new IntakeMechanism(this);
        outtakeMechanism = new OuttakeMechanism(this);


        headingController.setInputBounds(-Math.PI, Math.PI);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        drive.vectorMove(-gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.left_trigger - gamepad2.right_trigger, 0.6);

//        // Basic movement
//
//        double ly_I = 0;
//        double lx_I = 0;
//        double rx_I = 0;
//
//        if(Math.abs(-gamepad2.left_stick_y) > ly_I)
//            ly_I = Math.abs(-gamepad2.left_stick_y);
//        else
//            ly_I = (ly_I - Math.abs(-gamepad2.left_stick_y)) / 2;
//
//
//
//        if(Math.abs(gamepad2.left_stick_x) > lx_I)
//            lx_I = Math.abs(gamepad2.left_stick_x);
//        else
//            lx_I = (lx_I - Math.abs(gamepad2.left_stick_x)) / 2;
//
//
//        if(Math.abs(-gamepad2.left_trigger + gamepad2.right_trigger) > rx_I)
//            rx_I = Math.abs(-gamepad2.left_trigger + gamepad2.right_trigger);
//        else
//            rx_I = (rx_I - Math.abs(-gamepad2.left_trigger + gamepad2.right_trigger)) / 2;
////
//
//        drive.setWeightedDrivePower(new Pose2d(
//                ly_I * 0.6 * Math.signum(-gamepad2.left_stick_y),
//                0.6 * Math.signum(-gamepad2.left_stick_y),
//                rx_I * 0.6 * Math.signum(-gamepad2.left_trigger + gamepad2.right_trigger)
//        ));


        workflow();

//        if(gamepad1.y) {
//            intakeMechanism.DEB_ToggleIntermediary();
//            try {
//                Thread.sleep(700);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//
//        if(gamepad1.dpad_up) {
//            intakeMechanism.DEB_ToggleIntake();
//            try {
//                Thread.sleep(700);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//
//        if(gamepad1.dpad_down) {
//            intakeMechanism.DEB_ToggleServo();
//            try {
//                Thread.sleep(700);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }

//        circleFollow();
        telemetry.update();
//        bGamepad.update();

    }

    private Vector2d normalize(Vector2d vec) {
        double max = Math.max(vec.getX(), vec.getY());

        return new Vector2d(vec.getX() / max, vec.getY() / max);
    }

    private void circleFollow() {
//        if(Math.abs(gamepad1.right_stick_x) > 0.2|| Math.abs(gamepad1.right_stick_y) > 0.2) {
//
//            // courtesy of https://learnroadrunner.com
//
//            final Vector2d hubOrigin = new Vector2d(4, 0);
//
//            Pose2d driveDirection = new Pose2d();
//            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
//
//            // Create a vector from the gamepad x/y inputs which is the field relative movement
//            // Then, rotate that vector by the inverse of that heading for field centric control
//            Vector2d fieldFrameInput = new Vector2d(
//                    -gamepad1.right_stick_y,
//                    -gamepad1.right_stick_x
//            );
//            Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
//
//            // Difference between the target vector and the bot's position
//            Vector2d difference = hubOrigin.minus(poseEstimate.vec());
//            // Obtain the target angle for feedback and derivative for feedforward
//            double theta = difference.angle();
//
//            // Not technically omega because its power. This is the derivative of atan2
//            double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());
//
//            // Set the target heading for the heading controller to our desired angle
//            headingController.setTargetPosition(theta);
//
//            // Set desired angular velocity to the heading controller output + angular
//            // velocity feedforward
//            double headingInput = (headingController.update(poseEstimate.getHeading())
//                    * DriveConstants.kV + thetaFF)
//                    * DriveConstants.TRACK_WIDTH;
//
//            // Combine the field centric x/y velocity with our derived angular velocity
//            driveDirection = new Pose2d(
//                    robotFrameInput,
//                    headingInput
//            );
//
//            drive.setWeightedDrivePower(driveDirection);
//        }
    }

    @Override
    public void stop() {

    }

    //////// Intake + Outtake worflow

    enum WorkState {
        RESET,
        INTAKE_RUNNING,
        SELECT_LEVEL_AND_CONFIRM,
    }

    public void workflow() {
        telemetry.addData("WORK STATE", state.get().toString());
        switch (state.get()) {
            case RESET: {
                if(gamepad1.a) {
                    state.compareAndSet(WorkState.RESET, WorkState.INTAKE_RUNNING);
                    intakeMechanism.startWorkAsync();
                } else
                break;
            }
            case INTAKE_RUNNING: {
                if(intakeMechanism.workHasFinished()) {
                    boolean caughtPiece = intakeMechanism.getLastResult();
                    if(caughtPiece) {
                        gamepad1.rumbleBlips(1);
                        state.compareAndSet(WorkState.INTAKE_RUNNING, WorkState.SELECT_LEVEL_AND_CONFIRM);
                    } else {
                        gamepad1.rumbleBlips(2);
                        state.set(WorkState.RESET);
                    }
                } else {
                    if(gamepad1.x) {
                        intakeMechanism.interruptWork();
                        state.set(WorkState.SELECT_LEVEL_AND_CONFIRM);
                    }
                    if(gamepad1.b) {
                        intakeMechanism.toggleDirection();
                    }
                }

                break;
            }
            case SELECT_LEVEL_AND_CONFIRM: {
                if(gamepad1.b) {
                    outtakeMechanism.setStateAsync(OuttakeMechanism.State.LOADING);
                    state.compareAndSet(WorkState.SELECT_LEVEL_AND_CONFIRM, WorkState.RESET);
                } else if(gamepad1.dpad_down) {
                    outtakeMechanism.setStateAsync(OuttakeMechanism.State.LOW);
                } else if(gamepad1.dpad_left || gamepad1.dpad_right) {
                    outtakeMechanism.setStateAsync(OuttakeMechanism.State.MID);
                } else if(gamepad1.dpad_up) {
                    outtakeMechanism.setStateAsync(OuttakeMechanism.State.HIGH);
                }
            }
        }
    }
}
