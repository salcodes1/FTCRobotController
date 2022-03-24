package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutoRobot;

import java.util.HashMap;

public class RunParallelWait extends Action {

    HashMap<Action, Boolean> hasFinished = new HashMap<>();

    public RunParallelWait(Action... actions) {
        this.childActions = actions;
    }

    @Override
    public void _start(AutoRobot context) {
        hasFinished = new HashMap<>();
        for (Action action : childActions) {
            action.start(context);
        }
    }

    @Override
    public void _tick(AutoRobot context) {
        for (Action action : childActions) {
            if(!hasFinished.getOrDefault(action, false)) {
                if(action.hasFinished(context)) {
                    hasFinished.put(action, true);
                    action.end(context);
                } else action.tick(context);
            }
        }
    }

    @Override
    public boolean _hasFinished(AutoRobot context) {
        int matches = 0;

        for(boolean finished : hasFinished.values()) {
            if(finished) matches++;
        }

        return matches == childActions.length;
    }

    @Override
    public void _end(AutoRobot context) {
        for (Action a : childActions) {
            if(!a.hasFinished(context)) {
                a.end(context);
            }
        }
    }

}
