package org.firstinspires.ftc.teamcode.bt;

public abstract class ActionGroup extends Action {

    public ActionGroup() {
        childActions = new Action[]{constructAction()};
    }

    @Override
    public void _start(AutoRobot context) {
        childActions[0].start(context);
    }

    @Override
    public void _tick(AutoRobot context) {
        childActions[0].tick(context);
    }

    @Override
    public boolean _hasFinished(AutoRobot context) {
        return childActions[0].hasFinished(context);
    }

    @Override
    public void _end(AutoRobot context) {
        childActions[0].end(context);
    }

    protected abstract Action constructAction();

    @Override
    public boolean DEBUG_showChildren() {
        return false;
    }
}
