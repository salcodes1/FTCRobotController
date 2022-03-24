package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutoRobot;

public class RunDelay extends Action {

    long waitMs;
    long targetTimeMs;

    public RunDelay(long waitMs) {
        this.waitMs = waitMs;
    }

    @Override
    public void _start(AutoRobot context) {
        targetTimeMs = System.currentTimeMillis() + waitMs;
    }

    @Override
    public void _tick(AutoRobot context) {

    }

    @Override
    public boolean _hasFinished(AutoRobot context) {
        return targetTimeMs <= System.currentTimeMillis();
    }

    @Override
    public void _end(AutoRobot context) {

    }
}
