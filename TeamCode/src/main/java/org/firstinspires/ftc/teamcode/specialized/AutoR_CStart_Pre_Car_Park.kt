package org.firstinspires.ftc.teamcode.specialized

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.RR.util.AssetsTrajectoryManager
import org.firstinspires.ftc.teamcode.bt.Action
import org.firstinspires.ftc.teamcode.bt.actions.RunCarousel
import org.firstinspires.ftc.teamcode.bt.actions.RunTrajectory
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunLinear
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunParallelWait
import org.firstinspires.ftc.teamcode.bt.actions.outtake.OuttakeSetLevel

@Autonomous(name = "Red C Pre Car Park", preselectTeleOp = "TeleOp Blue")
class AutoR_CStart_Pre_Car_Park : AutoOpV4Base() {

    var hub_to_carousel : Trajectory? = null
    var carousel_to_warehouse : Trajectory? = null


    override fun precompileTrajectories() {
        side = Side.RED
        startLocation = StartLocation.CAROUSEL

        start_to_hub = AssetsTrajectoryManager.load(SIDE("cstart_to_hub"))
        hub_to_carousel = AssetsTrajectoryManager.load(SIDE("hub_to_carousel"))
        carousel_to_warehouse = AssetsTrajectoryManager.load(SIDE("carousel_to_warehouse"))
        hub_to_park = AssetsTrajectoryManager.load(SIDE("hub_to_park_bridge"))

    }

    override fun getRoutine(): Action =
        RunLinear(
            RunParallelWait(
                OuttakeSetLevel(preloadLevel),
                RunTrajectory(start_to_hub),
            ),
            RunTrajectory(hub_to_carousel),
            RunCarousel(2125, 0.25),
            DoNCycles(4, wPoints, carousel_to_warehouse, side),
            RunTrajectory(hub_to_park)
        )
}