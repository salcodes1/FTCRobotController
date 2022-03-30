package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class RunLinear extends Action {

    int actionIndex;

    public RunLinear(Action... actions) {
        this.childActions = actions;
    }

    @Override
    public void _start(AutonomousOpMode context) {
        actionIndex = 0;
        childActions[actionIndex].start(context);
    }

    @Override
    public void _tick(AutonomousOpMode context) {
        if(childActions[actionIndex].hasFinished(context)) {
            childActions[actionIndex].end(context);
            actionIndex++;

            if(!hasFinished(context))
                childActions[actionIndex].start(context);
        } else childActions[actionIndex].tick(context);
    }

    @Override
    public boolean _hasFinished(AutonomousOpMode context) {
        return actionIndex >= childActions.length;
    }

    @Override
    public void _end(AutonomousOpMode context) {
        // when finishing forcibly (e.g. RunParallelRace)
        if(!hasFinished(context))
            childActions[actionIndex].end(context);
    }
}
