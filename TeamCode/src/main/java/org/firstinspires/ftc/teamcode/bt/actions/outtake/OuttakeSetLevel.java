package org.firstinspires.ftc.teamcode.bt.actions.outtake;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class OuttakeSetLevel extends Action {

        private Outtake.Level level;

        public OuttakeSetLevel(Outtake.Level level) {
            this.level = level;
        }

        @Override
        public void _start(AutonomousOpMode context) {
            context.outtake.setLevel(level);
        }

        @Override
        public void _tick(AutonomousOpMode state) { }

        @Override
        public boolean _hasFinished(AutonomousOpMode state) {
            return state.outtake.hasFinished();
        }

        @Override
        public void _end(AutonomousOpMode state) {
//            state.outtake.motor.setPower(0);
        }
    }

