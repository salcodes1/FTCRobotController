package org.firstinspires.ftc.teamcode.bt.actions.intake;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class IntakeSetExtender extends Action {

    private boolean state;

    public IntakeSetExtender(boolean state) {

        this.state = state;
    }
    @Override
    protected void _start(AutonomousOpMode context) {
        context.intake.servoIntake.setPosition(state? 1 : 0);
    }

    @Override
    protected void _tick(AutonomousOpMode context) {

    }

    @Override
    protected boolean _hasFinished(AutonomousOpMode context) {
        return true;
    }

    @Override
    protected void _end(AutonomousOpMode context) {

    }
}
