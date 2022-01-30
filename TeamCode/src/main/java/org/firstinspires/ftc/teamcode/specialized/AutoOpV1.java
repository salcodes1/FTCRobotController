package org.firstinspires.ftc.teamcode.specialized;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RR.util.AssetsTrajectoryManager;

import java.util.Map;

@Config
@Autonomous
public class AutoOpV1 extends LinearOpMode {

    public static int CASE = 1;

    SampleMecanumDrive drive;

    Trajectory warehouse_to_hub, hub_to_warehouse;

    @Override
    public void runOpMode() throws InterruptedException {
        runInit();
        waitForStart();
        drive = new SampleMecanumDrive(hardwareMap);

        Thread t = new Thread(() -> {
            switch (CASE) {
                case 1: {
                    try {
                        case1();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    break;
                }
                case 2: {
                    case2();
                    break;
                }
                case 3: {
                    case3();
                    break;
                }
                default: {
                    break;
                }
            }
        });
        t.start();

        while(!isStopRequested())
        {
            Thread.yield();
        }

        t.interrupt();

    }

    private void case1() throws InterruptedException {
        drive.setPoseEstimate(new Pose2d(48, -66.59, Math.toRadians(0)));
        while(!Thread.interrupted()) {
            drive.followTrajectory(warehouse_to_hub);
            Thread.sleep(1000);
            drive.followTrajectory(hub_to_warehouse);
            Thread.sleep(1000);
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    }

    private void case2() {
        drive.setPoseEstimate(new Pose2d(-12.7, -42.5, Math.toRadians(-90)));
        drive.followTrajectory(hub_to_warehouse);
    }

    private void case3() {

    }

    private void runInit() {
        warehouse_to_hub = AssetsTrajectoryManager.load("warehouse_to_hub");
        hub_to_warehouse = AssetsTrajectoryManager.load("hub_to_warehouse");
    }




}
