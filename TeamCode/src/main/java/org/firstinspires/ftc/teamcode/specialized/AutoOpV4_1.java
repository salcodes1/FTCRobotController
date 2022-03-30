package org.firstinspires.ftc.teamcode.specialized;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.RR.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RR.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.actions.RunCarousel;
import org.firstinspires.ftc.teamcode.bt.actions.RunTrajectory;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunDelay;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunLinear;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelWait;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeDropFreight;
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeSetLevel;

@Autonomous(name = "Red C Pre Car Park Grace", preselectTeleOp = "TeleOp Blue")
public class AutoOpV4_1 extends AutoOpV4Base {

    TrajectorySequence hub_to_carousel;
    Trajectory carousel_to_warehouse, carousel_to_park;

    @Override
    protected void precompileTrajectories() {
       side = Side.RED;
       startLocation = StartLocation.CAROUSEL;

       start_to_hub = AssetsTrajectoryManager.load(SIDE("cstart_to_hub"));
       hub_to_carousel = drive.trajectorySequenceBuilder(new Pose2d(-12, -41, Math.toRadians(-90)))
           .setVelConstraint(new TranslationalVelocityConstraint(10))
           .addTrajectory(AssetsTrajectoryManager.load(SIDE("hub_to_carousel")))
           .build();
       carousel_to_warehouse = AssetsTrajectoryManager.load(SIDE("carousel_to_warehouse"));
       carousel_to_park = AssetsTrajectoryManager.load(SIDE("carousel_to_bridge"));

    }

    @Override
    protected Action getRoutine() {
        return new RunLinear(
            new RunParallelWait(
                new RunLinear(
                    new RunDelay(600),
                    new OuttakeSetLevel(preloadLevel)
                ),
                new RunTrajectory(start_to_hub)
            ),
            new OuttakeDropFreight(),
            new RunParallelWait(
                new RunLinear(
                    new RunDelay(300),
                    new OuttakeSetLevel(Outtake.Level.loading)
                ),
                new RunTrajectory(hub_to_carousel)
            ),
            new RunCarousel(-1800, 0.25),
//            new DoNCycles(4, new Vector2d(2.0, -0.5), carousel_to_warehouse),
            new RunTrajectory(carousel_to_park)
        );
    }
}
