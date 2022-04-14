package org.firstinspires.ftc.teamcode.specialized;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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
//@Disabled
@Autonomous(name = "Red Warehouse Cycles Park")
public class AutoOpV4_RedWarehouseCyclesPark extends AutoOpV4Base {

    @Override
    protected void setParams() {
        side = Side.RED;
    }

    @Override
    protected void precompileTrajectories() {
        startLocation = StartLocation.WAREHOUSE;

        hub_to_park = AssetsTrajectoryManager.load(SIDE("hub_to_park"));
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
                    new Vector2d(0, 0),
                    new Vector2d(0, 0),
                    new Vector2d(0, 0),
                    new Vector2d(0, 0)
                },
                //hub
                new Vector2d[] {
                    new Vector2d(0, 0),
                    new Vector2d(0, 0),
                    new Vector2d(0, 0),
                    new Vector2d(0, 0),
                    new Vector2d(0, 0)
                },
                side
            ),
            new RunParallelWait(
                new RunTrajectory(hub_to_park),
                new OuttakeSetLevel(Outtake.Level.loading)
            )
        );
    }
}
