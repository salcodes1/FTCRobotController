package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutoRobot;

public class RunLinear extends Action {

    int actionIndex;

    public RunLinear(Action... actions) {
        this.childActions = actions;
    }

    @Override
    public void _start(AutoRobot context) {
        actionIndex = 0;
        childActions[actionIndex].start(context);
    }

    @Override
    public void _tick(AutoRobot context) {
        if(childActions[actionIndex].hasFinished(context)) {
            childActions[actionIndex].end(context);
            actionIndex++;

            if(!_hasFinished(context))
                childActions[actionIndex].start(context);
        } else childActions[actionIndex].tick(context);
    }

    @Override
    public boolean _hasFinished(AutoRobot context) {
        return actionIndex >= childActions.length;
    }

    @Override
    public void _end(AutoRobot context) {
        // when finishing forcibly (e.g. RunParallelRace)
        if(!_hasFinished(context))
            childActions[actionIndex].end(context);
    }
}
