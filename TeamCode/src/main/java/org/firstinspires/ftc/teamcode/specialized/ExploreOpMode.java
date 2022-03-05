package org.firstinspires.ftc.teamcode.specialized;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Date;

@TeleOp
public class ExploreOpMode extends LinearOpMode {

    public long lastTime;
    public long lastDistance;
    public double lastVelocity;
    public double lastAcceleration;

    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotor.class, "intakeMotor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        lastTime = new Date().getTime();

        motor.setPower(-1);

        while(opModeIsActive() && !isStopRequested()) {
            long currentTime = new Date().getTime();
            long deltaTime = currentTime - lastTime;

            long currentDistance = motor.getCurrentPosition();
            long deltaDistance = currentDistance - lastDistance;

            double currentVelocity = (float)deltaDistance / deltaTime;
            double deltaVelocity = currentVelocity - lastVelocity;

            double currentAcceleration = deltaVelocity / deltaTime;

            telemetry.addData("vel", currentVelocity);
            telemetry.addData("accel", currentAcceleration);
            telemetry.update();

            lastTime = currentTime;
            lastDistance = currentDistance;
            lastVelocity = currentVelocity;
            lastAcceleration = currentAcceleration;
        }
    }
}
