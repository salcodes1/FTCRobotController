package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class RunWhile extends Action {

    public interface Cond {
        boolean call(AutonomousOpMode state);
    }

    Cond cond;

    public RunWhile(Cond cond) {
        this.cond = cond;
    }

    @Override
    public void _start(AutonomousOpMode context) { }

    @Override
    public void _tick(AutonomousOpMode context) { }

    @Override
    public boolean _hasFinished(AutonomousOpMode context) {
        return !cond.call(context);
    }

    @Override
    public void _end(AutonomousOpMode context) { }

}
