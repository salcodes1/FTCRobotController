package org.firstinspires.ftc.teamcode.specialized;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class OpModeDumb extends OpMode {

    DigitalChannel touchSensor;


    @Override
    public void init() {
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        telemetry.addData("touchSensor", touchSensor.getState()? "true": "false");
    }
}
