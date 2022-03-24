package org.firstinspires.ftc.teamcode.bt.actions.outtake;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class OuttakeDropFreight extends Action {

    int time = 200;

    public OuttakeDropFreight() { }

    public OuttakeDropFreight(int time) {
        this.time = time;
    }

    @Override
    public void _start(AutonomousOpMode context) {


    }

    @Override
    public void _tick(AutonomousOpMode state) {

    }

    @Override
    public boolean _hasFinished(AutonomousOpMode state) {
        return true;
    }

    @Override
    public void _end(AutonomousOpMode state) {

    }
}
