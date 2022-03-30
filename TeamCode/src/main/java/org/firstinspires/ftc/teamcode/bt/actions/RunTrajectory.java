package org.firstinspires.ftc.teamcode.bt.actions;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class RunTrajectory extends Action {

    private Trajectory trajectory;
    private TrajectorySequence trajectorySequence;

    public RunTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
    }


    public RunTrajectory(TrajectorySequence trajectorySequence) {
        this.trajectorySequence = trajectorySequence;
    }

    @Override
    public void _start(AutonomousOpMode context) {
        if(context.drive.isBusy()) {
            throw new RuntimeException("RunTrajectory: You are giving a new trajectory " +
                    "when the current one isn't done!");
            // For debuggability purposes
        }
        if(trajectory != null)
            context.drive.followTrajectoryAsync(trajectory);
        else
            context.drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    @Override
    public void _tick(AutonomousOpMode state) {
        // There isn't anything to do on a tick-by-tick basis.
    }

    @Override
    public boolean _hasFinished(AutonomousOpMode state) {
        return !state.drive.isBusy();
        // The action is finished when the robot's drive is no longer moving.
    }

    @Override
    public void _end(AutonomousOpMode state) {
        // There isn't any cleanup to do.
    }
}
