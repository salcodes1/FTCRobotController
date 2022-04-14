package org.firstinspires.ftc.teamcode.specialized;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunDelay;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelWait;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeSetPower;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeWaitForElement;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntermediarySetRunning;

@Disabled
@Autonomous(name="AutoDumb")
public class AutoDumb extends OpMode {

    Rev2mDistanceSensor distanceSensor;
    double lastValue;
    long lastMs;
    long delta;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
    }

    @Override
    public void start() {
        super.start();
        lastMs = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        double currentValue = distanceSensor.getDistance(DistanceUnit.MM);
        if(currentValue != lastValue) {
            lastValue = currentValue;
            delta = System.currentTimeMillis() - lastMs;
            lastMs = System.currentTimeMillis();
        }
        telemetry.addData("dist", lastValue);
        telemetry.addData("delta", delta);
    }
}
