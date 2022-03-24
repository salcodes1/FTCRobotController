package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutoRobot;

public class RunInline extends Action {

    private Exec exec;

    public interface Exec {
        void call(AutoRobot context);
    }

    public RunInline(Exec exec) {
        this.exec = exec;
    }

    @Override
    public void _start(AutoRobot context) {
        exec.call(context);
    }

    @Override
    public void _tick(AutoRobot context) {

    }

    @Override
    public boolean _hasFinished(AutoRobot context) {
        return false;
    }

    @Override
    public void _end(AutoRobot context) {

    }
}
