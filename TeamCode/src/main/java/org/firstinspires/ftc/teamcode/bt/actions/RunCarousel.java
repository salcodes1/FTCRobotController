package org.firstinspires.ftc.teamcode.bt.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class RunCarousel extends Action {

    private int targetPosition;
    private double power;

    public RunCarousel(int targetPosition, double power) {
        this.targetPosition = targetPosition;
        this.power = power;
    }

    @Override
    public void _start(AutonomousOpMode context) {
        context.carouselMotor.setTargetPosition(targetPosition);
        context.carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        context.carouselMotor.setPower(power);
    }

    @Override
    public void _tick(AutonomousOpMode state) {

    }

    @Override
    public boolean _hasFinished(AutonomousOpMode state) {
        return !state.carouselMotor.isBusy();
    }

    @Override
    public void _end(AutonomousOpMode state) {
        state.carouselMotor.setPower(0);
    }
}
