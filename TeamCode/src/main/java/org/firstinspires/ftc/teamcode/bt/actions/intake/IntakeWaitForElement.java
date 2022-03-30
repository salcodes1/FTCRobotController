package org.firstinspires.ftc.teamcode.bt.actions.intake;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

import java.util.Arrays;

public class IntakeWaitForElement extends Action {
    @Override
    protected void _start(AutonomousOpMode context) {
//        context.intake.servoIntake.setPosition(1);
    }

    @Override
    protected void _tick(AutonomousOpMode context) {

    }

    @SuppressLint("NewApi")
    @Override
    protected boolean _hasFinished(AutonomousOpMode context) {
        int s = 0;

        for(DigitalChannel touchSensor : context.intake.touchSensors) {
            if(touchSensor.getState()) s++;
        }

        return s > 0;
    }

    @Override
    protected void _end(AutonomousOpMode context) {
        context.intake.servoIntake.setPosition(0);
    }

    @Override
    public String getCustomDisplay(AutonomousOpMode context) {
        int s = 0;
        for(DigitalChannel touchSensor : context.intake.touchSensors) {
            if(touchSensor.getState()) s++;
        }
        return s + "/" + context.intake.touchSensors.length;
    }
}
