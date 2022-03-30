package org.firstinspires.ftc.teamcode.specialized.prev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class OpModeDumb extends OpMode {

    DigitalChannel touchSensor;

    DcMotor m1, m2, m3, m4;

    @Override
    public void init() {
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);


        m1 = hardwareMap.get(DcMotor.class, "motorBL");
        m2 = hardwareMap.get(DcMotor.class, "motorBR");
        m3 = hardwareMap.get(DcMotor.class, "motorFL");
        m4 = hardwareMap.get(DcMotor.class, "motorFR");
    }

    @Override
    public void loop() {
        telemetry.addData("touchSensor", touchSensor.getState()? "true": "false");
        telemetry.addData("motorBL", m1.getCurrentPosition());
        telemetry.addData("motorBR", m2.getCurrentPosition());
        telemetry.addData("motorFL", m3.getCurrentPosition());
        telemetry.addData("motorFR", m4.getCurrentPosition());

    }
}
