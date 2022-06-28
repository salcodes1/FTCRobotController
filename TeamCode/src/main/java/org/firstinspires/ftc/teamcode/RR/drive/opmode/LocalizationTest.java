package org.firstinspires.ftc.teamcode.RR.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    Servo sl, sc, sr;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor l, c, r;
        l = hardwareMap.get(DcMotor.class, "motorFL");
        c = hardwareMap.get(DcMotor.class, "motorFR");
        r = hardwareMap.get(DcMotor.class, "motorBL");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sl = hardwareMap.get(Servo.class, "servoOdoLeft");
        sc = hardwareMap.get(Servo.class, "servoOdoCenter");
        sr = hardwareMap.get(Servo.class, "servoOdoRight");
        sl.setPosition(0.0);
        sc.setPosition(1.0);
        sr.setPosition(1.0);
        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));

            AngularVelocity ang = imu.getAngularVelocity();


            telemetry.addData("Current IMU Ang Velo Deg X", Math.toDegrees(ang.xRotationRate));
            telemetry.addData("Current IMU Ang Velo Deg Y", Math.toDegrees(ang.yRotationRate));
            telemetry.addData("Current IMU Ang Velo Deg Z", Math.toDegrees(ang.zRotationRate));
            telemetry.addData("Current IMU Ang Velo Unit", ang.unit);
            telemetry.addData("Left", l.getCurrentPosition());
            telemetry.addData("Center", c.getCurrentPosition());
            telemetry.addData("Right", r.getCurrentPosition());
            telemetry.update();
        }
    }
}
