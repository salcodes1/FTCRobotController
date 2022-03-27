package org.firstinspires.ftc.teamcode.bt.actions.sequences;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.ActionGroup;
import org.firstinspires.ftc.teamcode.bt.actions.RunTrajectory;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunDelay;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunLinear;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelRace;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelWait;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeSetExtender;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeSetRunning;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeWaitForElement;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntermediarySetRunning;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeDropFreight;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeSetLevel;

public class DoCycle extends ActionGroup {

    private final Trajectory h_to_w;
    private final Trajectory w_to_h;

    public DoCycle(Trajectory h_to_w, Trajectory w_to_h) {
        this.h_to_w = h_to_w;
        this.w_to_h = w_to_h;
    }

    @Override
    protected Action constructGroup() {
        return new RunLinear(
            new RunParallelWait(
                new OuttakeSetLevel(Outtake.Level.loading),
                new RunTrajectory(h_to_w),
                new IntakeSetRunning(true),
                new IntermediarySetRunning(true),
                new RunParallelRace(
                    new IntakeWaitForElement(),
                    new RunDelay(3500)
                )
            ),
            new RunDelay(200),
            new IntakeSetRunning(false),
            new RunParallelWait(
                new RunTrajectory(w_to_h),
                new RunLinear(
                    new IntermediarySetRunning(false),
                    new RunDelay(900),
                    new OuttakeSetLevel(Outtake.Level.high)
                )
            ),
            new OuttakeDropFreight()
        );

    }
}
