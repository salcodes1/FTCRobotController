package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class RunRepeated extends Action {

    private int times;
    int index;

    public RunRepeated(int times, Action action) {
        this.childActions = new Action[]{action};
        this.times = times;
    }
    @Override
    public void _start(AutonomousOpMode context) {
        childActions[0].start(context);
    }

    @Override
    public void _tick(AutonomousOpMode context) {
        childActions[0].tick(context);
        if(childActions[0].hasFinished(context)) {
            childActions[0].end(context);
            index++;
            childActions[0].start(context);
        }
    }

    @Override
    public boolean _hasFinished(AutonomousOpMode context) {
        return index >= times;
    }

    @Override
    public void _end(AutonomousOpMode context) {
        childActions[0].end(context);
    }
}
