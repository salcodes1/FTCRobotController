package org.firstinspires.ftc.teamcode.bt;

public abstract class ComposedAction extends Action {


    @Override
    public void _start(AutonomousOpMode context) {
        childActions = new Action[]{constructGroupAtStart(context)};
        childActions[0].start(context);
    }

    @Override
    public void _tick(AutonomousOpMode context) {
        childActions[0].tick(context);
    }

    @Override
    public boolean _hasFinished(AutonomousOpMode context) {
        return childActions[0].hasFinished(context);
    }

    @Override
    public void _end(AutonomousOpMode context) {
        childActions[0].end(context);
    }

    protected abstract Action constructGroupAtStart(AutonomousOpMode context);

    @Override
    public boolean DEBUG_showChildren() {
        return true;
    }
}
