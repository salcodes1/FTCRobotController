package org.firstinspires.ftc.teamcode.bt.actions.intake;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class IntakeSetRunning extends Action {

    private boolean run;

    public IntakeSetRunning(boolean run) {
        this.run = run;
    }

    @Override
    public void _start(AutonomousOpMode context) {
        if(run) context.intake.intakeMotor.setPower(1);
        else context.intake.intakeMotor.setPower(0);
    }

    @Override
    public void _tick(AutonomousOpMode state) { }

    @Override
    public boolean _hasFinished(AutonomousOpMode state) {
        return true;
    }

    @Override
    public void _end(AutonomousOpMode state) { }
}
