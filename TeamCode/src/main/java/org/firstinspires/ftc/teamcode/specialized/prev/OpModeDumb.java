package org.firstinspires.ftc.teamcode.specialized.prev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class OpModeDumb extends OpMode {

    DigitalChannel touchSensor1, touchSensor2, touchSensor3;

    @Override
    public void init() {
        touchSensor2 = hardwareMap.get(DigitalChannel.class, "touchSensor2");
        touchSensor1 = hardwareMap.get(DigitalChannel.class, "touchSensor1");
        touchSensor3 = hardwareMap.get(DigitalChannel.class, "touchSensor3");
        touchSensor1.setMode(DigitalChannel.Mode.INPUT);
        touchSensor2.setMode(DigitalChannel.Mode.INPUT);
        touchSensor3.setMode(DigitalChannel.Mode.INPUT);

    }

    @Override
    public void loop() {
        telemetry.addData("touchSensor1", touchSensor1.getState()? "true": "false");
        telemetry.addData("touchSensor2", touchSensor2.getState()? "true": "false");
        telemetry.addData("touchSensor3", touchSensor3.getState()? "true": "false");

    }
}
