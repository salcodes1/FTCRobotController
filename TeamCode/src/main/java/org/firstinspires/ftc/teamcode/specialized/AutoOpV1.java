package org.firstinspires.ftc.teamcode.specialized;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RR.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeMechanism;
import org.firstinspires.ftc.teamcode.statics.PoseStorage;

@Config
@Autonomous
public class AutoOpV1 extends LinearOpMode {

    enum eAutoState {
        START_TO_CAROUSEL,
        CAROUSEL_RUNNING,
        CAROUSEL_TO_HUB,
        START_TO_HUB,
        HUB_TO_WAREHOUSE,
        WAREHOUSE_TO_HUB,
        INTAKE_RUNNING,
        OUTTAKE_RUNNING,
        IDLE,
        HUB_TO_PARK
    }
    public static int CASE = 1;

    boolean ciclu;

    SampleMecanumDrive drive;

    Trajectory warehouse_to_hub, hub_to_warehouse, start_to_hub, start_to_carousel, carousel_to_hub, hub_to_park;

    IntakeMechanism intakeMechanism;
    OuttakeMechanism outtakeMechanism;

    DcMotor carouselMotor;
    Servo containerServo;

    eAutoState state;

    @Override
    public void runOpMode() throws InterruptedException {
        runInit();
        waitForStart();



        state = eAutoState.START_TO_CAROUSEL;
        drive.setPoseEstimate(PoseStorage.poseEstimate);
        drive.followTrajectoryAsync(start_to_carousel);


        while(opModeIsActive() && !isStopRequested())
        {
            run();
            telemetry.update();
        }

    }

    private void run() {


        switch (state) {
            case IDLE: {
                // do Nothing
                break;
            }

            case START_TO_CAROUSEL: {
                if(!drive.isBusy()) {
                    carouselMotor.setTargetPosition(-800);
                    carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carouselMotor.setPower(-0.17);
                    state = eAutoState.CAROUSEL_RUNNING;
                    outtakeMechanism.setStateAsync(OuttakeMechanism.State.HIGH);
                }
                break;
            }
            case CAROUSEL_RUNNING: {
                if(!carouselMotor.isBusy()) {
                    carouselMotor.setPower(0);
                    drive.followTrajectoryAsync(carousel_to_hub);
                    state = eAutoState.CAROUSEL_TO_HUB;
                }
                break;
            }
            case CAROUSEL_TO_HUB: {
                if(!drive.isBusy()) {
                    outtakeMechanism.setStateAsync(OuttakeMechanism.State.LOADING);
                    drive.followTrajectoryAsync(hub_to_warehouse);
                    state = eAutoState.HUB_TO_WAREHOUSE;
                }
                break;
            }
            case HUB_TO_WAREHOUSE: {
                if(!drive.isBusy()) {
                    if(!ciclu)
                    {
                        intakeMechanism.startWorkAsync(2000);
                        state = eAutoState.INTAKE_RUNNING;
                        ciclu = true;
                    } else state = eAutoState.IDLE;
                }
                break;
            }
            case INTAKE_RUNNING: {
                if(intakeMechanism.workHasFinished() || intakeMechanism.getLastResult()) {
                    drive.followTrajectoryAsync(warehouse_to_hub);
                    state = eAutoState.WAREHOUSE_TO_HUB;
                }
                break;
            }
            case WAREHOUSE_TO_HUB: {
                if(!drive.isBusy()) {
                    outtakeMechanism.setStateAsync(OuttakeMechanism.State.HIGH);
                    state = eAutoState.OUTTAKE_RUNNING;
                }
                break;
            }
            case OUTTAKE_RUNNING: {
                outtakeMechanism.setStateAsync(OuttakeMechanism.State.LOADING);
                if(!ciclu) {
                    drive.followTrajectoryAsync(hub_to_warehouse);
                    state = eAutoState.HUB_TO_WAREHOUSE;
                }
                else {
                    drive.followTrajectoryAsync(hub_to_park);
                    state = eAutoState.HUB_TO_PARK;
                }

                break;


            }
            case HUB_TO_PARK:
                break;
        }
        drive.update();
        PoseStorage.poseEstimate = drive.getPoseEstimate();
    }

    private void case1() {
        drive.setPoseEstimate(new Pose2d(48, -66.59, Math.toRadians(0)));
    }



    private void runInit() {
        drive = new SampleMecanumDrive(hardwareMap);
        intakeMechanism = new IntakeMechanism(this);
        outtakeMechanism = new OuttakeMechanism(this);
        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        containerServo = hardwareMap.get(Servo.class, "containerServo");


        PoseStorage.poseEstimate = new Pose2d(-36.00, -63.34, Math.toRadians(90));
//        PoseStorage.poseEstimate = new Pose2d(48, -66.59, 0);

        warehouse_to_hub = AssetsTrajectoryManager.load("warehouse_to_hub");
        hub_to_warehouse = AssetsTrajectoryManager.load("hub_to_warehouse");
        start_to_carousel = AssetsTrajectoryManager.load("start_to_carousel");
        carousel_to_hub = AssetsTrajectoryManager.load("carousel_to_hub");
        hub_to_park = AssetsTrajectoryManager.load("hub_to_park");


//        start_to_hub = AssetsTrajectoryManager.load("start_to_hub");
        start_to_hub = drive.trajectoryBuilder(PoseStorage.poseEstimate, true)
                .lineTo(new Vector2d(-12, -36))
                .build();

        CASE = 1; // TODO: opencv


    }




}
