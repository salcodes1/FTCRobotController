package org.firstinspires.ftc.teamcode.bt.actions.intake;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

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
        return context.intake.freightDetectionPipeline.freightInside();
    }

    @Override
    protected void _end(AutonomousOpMode context) {
        context.intake.servoIntake.setPosition(0);
    }

}
