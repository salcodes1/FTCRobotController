package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutoRobot;

public class RunRepeated extends Action {

    private Action action;
    private int times;
    int index;

    public RunRepeated(int times, Action action) {
        this.action = action;
        this.times = times;
    }
    @Override
    public void _start(AutoRobot context) {
        action.start(context);
    }

    @Override
    public void _tick(AutoRobot context) {
        action.tick(context);
        if(action.hasFinished(context)) {
            action.end(context);
            index++;
            action.start(context);
        }
    }

    @Override
    public boolean _hasFinished(AutoRobot context) {
        return index >= times;
    }

    @Override
    public void _end(AutoRobot context) {
        action.end(context);
    }
}
