package org.firstinspires.ftc.teamcode.bt.actions.outtake;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutoRobot;

public class OuttakeDropFreight extends Action {

    int time = 200;

    public OuttakeDropFreight() { }

    public OuttakeDropFreight(int time) {
        this.time = time;
    }

    @Override
    public void _start(AutoRobot context) {


    }

    @Override
    public void _tick(AutoRobot state) {

    }

    @Override
    public boolean _hasFinished(AutoRobot state) {
        return true;
    }

    @Override
    public void _end(AutoRobot state) {

    }
}
