package org.firstinspires.ftc.teamcode.specialized;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
//    @Override
//    protected void precompileTrajectories() {
//
//    }
//
//    @Override
//    protected Action getRoutine() {
//        return new RunParallelWait(
//            new IntakeSetPower(-1),
//            new IntermediarySetRunning(true),
//            new RunDelay(30000),
//            new IntakeWaitForElement()
//        );
//    }

    Outtake outtake;

    @Override
    public void init() {
        outtake = new Outtake(this);
    }

    @Override
    public void loop() {
        Log.d("XX", outtake.motor.getCurrentPosition() + "/" + outtake.motor.getTargetPosition());
    }
}
