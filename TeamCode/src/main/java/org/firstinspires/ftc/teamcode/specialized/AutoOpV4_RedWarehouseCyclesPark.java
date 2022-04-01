package org.firstinspires.ftc.teamcode.specialized;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.RR.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.actions.RunTrajectory;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunAsync;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunInline;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunLinear;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelWait;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeDropFreight;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeSetLevel;

@Autonomous(name = "Red Warehouse Cycles Park")
public class AutoOpV4_RedWarehouseCyclesPark extends AutoOpV4Base {

    @Override
    protected void setParams() {
        side = Side.RED;
    }

    @Override
    protected void precompileTrajectories() {
        startLocation = StartLocation.WAREHOUSE;

        start_to_hub = AssetsTrajectoryManager.load(SIDE("wstart_to_hub"));

    }

    @Override
    protected Action getRoutine() {
        return new RunLinear(
            new RunInline(ctx -> ctx.outtake.capServo.setPosition(0.65)),
            new RunParallelWait(
                new RunTrajectory(start_to_hub),
                new OuttakeSetLevel(preloadLevel)
            ),
            new OuttakeDropFreight(),
            new DoNCycles(
                4,
                //warehouse
                new Vector2d[] {
                    new Vector2d(4, 0),
                    new Vector2d(6.5, -1),
                    new Vector2d(9, -2),
                        new Vector2d(0, 0 )
                },
                //hub
                new Vector2d[] {
                    new Vector2d(0, -2),
                    new Vector2d(0, -2),
                    new Vector2d(0, -2),
                        new Vector2d(0, 0)
                },
                side
            ),
            new RunAsync((ctx) -> hub_to_park = drive.trajectoryBuilder(drive.getPoseEstimate(), 0)
                .splineToConstantHeading(new Vector2d(-60, -35), 0)
                .build()),
            new RunParallelWait(
                new RunTrajectory(hub_to_park),
                new OuttakeSetLevel(Outtake.Level.loading)
            )
        );
    }
}
