package org.firstinspires.ftc.teamcode.specialized;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.actions.RunTrajectory;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunDelay;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunLinear;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelRace;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelWait;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeSetExtender;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeSetPower;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntakeWaitForElement;
import org.firstinspires.ftc.teamcode.bt.actions.intake.IntermediarySetRunning;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeDropFreight;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeSetLevel;

@Disabled
@Autonomous
public class AutoFreightDetectionTest extends AutoOpV4Base{
    @Override
    protected void precompileTrajectories() {

    }

    @Override
    protected Action getRoutine() {

        return new RunLinear(
            new RunParallelWait(
                new IntakeSetPower(-1),
                new IntermediarySetRunning(true),
                new IntakeSetExtender(1.0),
                new OuttakeSetLevel(Outtake.Level.loading),
                new RunLinear(
                    new RunParallelRace(
                        new IntakeWaitForElement(),
                        new RunDelay(6000)
                    ),
                    new IntakeSetPower(0),
                    new IntakeSetExtender(0)
                )

            ),
            new RunParallelWait(
                new RunLinear(
                    new RunDelay(600),
                    new IntakeSetPower(-1),
                    new RunDelay(1300),
                    new IntakeSetPower(1),
                    new IntermediarySetRunning(true),
                    new OuttakeSetLevel(Outtake.Level.high)
                )
            ),
            new OuttakeDropFreight()
        );
    }
}
