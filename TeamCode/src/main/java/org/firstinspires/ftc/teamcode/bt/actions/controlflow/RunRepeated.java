package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class RunRepeated extends Action {

    private Action action;
    private int times;
    int index;

    public RunRepeated(int times, Action action) {
        this.action = action;
        this.times = times;
    }
    @Override
    public void _start(AutonomousOpMode context) {
        action.start(context);
    }

    @Override
    public void _tick(AutonomousOpMode context) {
        action.tick(context);
        if(action.hasFinished(context)) {
            action.end(context);
            index++;
            action.start(context);
        }
    }

    @Override
    public boolean _hasFinished(AutonomousOpMode context) {
        return index >= times;
    }

    @Override
    public void _end(AutonomousOpMode context) {
        action.end(context);
    }
}
