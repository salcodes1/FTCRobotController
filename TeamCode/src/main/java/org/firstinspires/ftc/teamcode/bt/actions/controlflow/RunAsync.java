package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutoRobot;

public class RunAsync extends Action {

    interface Exec {
        void call(AutoRobot state);
    }

    Exec e;
    Thread execThread;

    public RunAsync(Exec e) {
        this.e = e;
    }

    @Override
    public void _start(AutoRobot context) {
        execThread = new Thread(() -> e.call(context));
        execThread.start();
    }

    @Override
    public void _tick(AutoRobot context) {
        // Should not have to tick
    }

    @Override
    public boolean _hasFinished(AutoRobot context) {
        return execThread != null && !execThread.isAlive();
    }

    @Override
    public void _end(AutoRobot context) {
        execThread.interrupt();
    }
}
