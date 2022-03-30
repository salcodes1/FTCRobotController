package org.firstinspires.ftc.teamcode.specialized;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunDelay;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelWait;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeSetPower;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeWaitForElement;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntermediarySetRunning;

@Autonomous(name="AutoDumb")
public class AutoDumb extends AutoOpV4Base {
    @Override
    protected void precompileTrajectories() {

    }

    @Override
    protected Action getRoutine() {
        return new RunParallelWait(
            new IntakeSetPower(-1),
            new IntermediarySetRunning(true),
            new RunDelay(30000),
            new IntakeWaitForElement()
        );
    }
}
