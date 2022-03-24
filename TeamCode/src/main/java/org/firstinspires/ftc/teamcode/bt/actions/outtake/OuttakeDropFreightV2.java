package org.firstinspires.ftc.teamcode.bt.actions.outtake;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.ActionGroup;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunDelay;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunInline;
import org.firstinspires.ftc.teamcode.bt.actions.controlflow.RunLinear;

public class OuttakeDropFreightV2 extends ActionGroup {
    @Override
    protected Action constructAction() {
        return new RunLinear(
            new RunInline((context) -> context.containerServo.setPosition(1)),
            new RunDelay(400),
            new RunInline((context) -> context.containerServo.setPosition(0))
        );
    }
}
