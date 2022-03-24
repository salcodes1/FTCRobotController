package org.firstinspires.ftc.teamcode.bt.actions.intake;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutoRobot;

public class IntakeSetRunning extends Action {

    private boolean run;

    public IntakeSetRunning(boolean run) {
        this.run = run;
    }

    @Override
    public void _start(AutoRobot context) {
        if(run) context.intake.work();
        else context.intake.stop();
    }

    @Override
    public void _tick(AutoRobot state) { }

    @Override
    public boolean _hasFinished(AutoRobot state) {
        return true;
    }

    @Override
    public void _end(AutoRobot state) { }
}
