package org.firstinspires.ftc.teamcode.specialized;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.FreightSensor;

@TeleOp(name = "Distance Test")
public class OpDistSens extends OpMode {

    Intake intake;
    FreightSensor freightSensor;
    int freight;
    boolean gate = false;
    long nextTimeout;

    @Override
    public void init() {
        intake = new Intake(this);
        freightSensor = new FreightSensor(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
        intake.work();
    }

    @Override
    public void loop() {
        freightSensor.update();
        if(System.currentTimeMillis() < nextTimeout) return;
        intake.work();
        if(freightSensor.isDetectingFreight()) {
            if(!gate) {
                freight++;
                gate = true;
                intake.stop();
                nextTimeout = System.currentTimeMillis() + 1000;
            }
        } else gate = false;
        telemetry.addData("freight", freight);
    }
}
