package org.firstinspires.ftc.teamcode.specialized;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class AutoWheelTest extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {

        DcMotor m1, m2, m3, m4;

        m1 = hardwareMap.get(DcMotor.class, "motorFL");
        m2 = hardwareMap.get(DcMotor.class, "motorBL");
        m3 = hardwareMap.get(DcMotor.class, "motorBR");
        m4 = hardwareMap.get(DcMotor.class, "motorFR");

        if(gamepad1.a)
            m1.setPower(1);
        else
            m1.setPower(0);

        if(gamepad1.b)
            m2.setPower(1);
        else
            m2.setPower(0);

        if(gamepad1.x)
            m3.setPower(1);
        else
            m3.setPower(0);

        if(gamepad1.y)
            m4.setPower(1);
        else
            m4.setPower(0);
    }
}
