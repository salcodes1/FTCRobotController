package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutoRobot;

public class RunWhile extends Action {

    public interface Cond {
        boolean call(AutoRobot state);
    }

    Cond cond;

    public RunWhile(Cond cond) {
        this.cond = cond;
    }

    @Override
    public void _start(AutoRobot context) { }

    @Override
    public void _tick(AutoRobot context) { }

    @Override
    public boolean _hasFinished(AutoRobot context) {
        return !cond.call(context);
    }

    @Override
    public void _end(AutoRobot context) { }

}
