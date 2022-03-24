package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class RunAsync extends Action {

    interface Exec {
        void call(AutonomousOpMode state);
    }

    Exec e;
    Thread execThread;

    public RunAsync(Exec e) {
        this.e = e;
    }

    @Override
    public void _start(AutonomousOpMode context) {
        execThread = new Thread(() -> e.call(context));
        execThread.start();
    }

    @Override
    public void _tick(AutonomousOpMode context) {
        // Should not have to tick
    }

    @Override
    public boolean _hasFinished(AutonomousOpMode context) {
        return execThread != null && !execThread.isAlive();
    }

    @Override
    public void _end(AutonomousOpMode context) {
        execThread.interrupt();
    }
}
