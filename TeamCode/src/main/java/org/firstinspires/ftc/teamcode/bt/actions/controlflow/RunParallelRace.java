package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class RunParallelRace extends Action {

    boolean finished;

    public RunParallelRace(Action... actions) {
        this.childActions = actions;
    }

    @Override
    public void _start(AutonomousOpMode context) {
        finished = false;
        for (Action action : childActions) {
            action.start(context);
        }
    }

    @Override
    public void _tick(AutonomousOpMode context) {
        for (Action action : childActions) {
            if(action.hasFinished(context)) {
                finished = true;
            }
        }

        if(finished) {
            for (Action a : childActions) {
                a.end(context);
            }
        } else {
            for (Action a : childActions) {
                a.tick(context);
            }
        }
    }

    @Override
    public boolean _hasFinished(AutonomousOpMode context) {
        return finished;
    }

    @Override
    public void _end(AutonomousOpMode context) {
        for (Action a : childActions) {
            a.end(context);
        }
    }

}

