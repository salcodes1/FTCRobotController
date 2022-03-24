package org.firstinspires.ftc.teamcode.bt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.statics.PoseStorage;

import java.io.StringWriter;
import java.util.ArrayList;

import hu.webarticum.treeprinter.ListingTreePrinter;
import hu.webarticum.treeprinter.SimpleTreeNode;

public class AutoRobot {

    Action currentAction;
    LinearOpMode opMode;

    public SampleMecanumDrive drive;
    public DcMotor carouselMotor;
    public Outtake outtake;
    public Intake intake;
    public Servo containerServo;


    public AutoRobot(LinearOpMode opMode) {
        this.opMode = opMode;

        drive = new SampleMecanumDrive(opMode.hardwareMap);

        carouselMotor = opMode.hardwareMap.get(DcMotor.class, "carouselMotor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtake = new Outtake(opMode);
        intake = new Intake(opMode);

        containerServo = opMode.hardwareMap.get(Servo.class, "containerServo");


        opMode.telemetry.setAutoClear(true);
    }

    public void start(Action initialAction) {
        drive.setPoseEstimate(PoseStorage.poseEstimate);

//        opMode.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        currentAction = initialAction;
        currentAction.start(this);

        while(opMode.opModeIsActive() && !opMode.isStopRequested()) {
            if(currentAction.hasFinished(this)) {
                currentAction.end(this);
                break;
            } else currentAction.tick(this);
            updateRobotState();
            printActionTree();

        }
    }

    private void printActionTree() {
        StringWriter w = new StringWriter();
        SimpleTreeNode root = Utils.GetTreeFromAction(currentAction);
        new ListingTreePrinter().print(root, w);

        opMode.telemetry.addLine(w.toString());
        opMode.telemetry.update();
    }

    void updateRobotState() {
        drive.update();
        outtake.update();

    }
}

class Utils {
    static SimpleTreeNode GetTreeFromAction(Action action) {

        SimpleTreeNode node = new SimpleTreeNode(
            action.getClass().getSimpleName() + "(" + action.getState().name() + ")"
        );
        if(action.getChildActions() != null)
            for(Action a : action.getChildActions()) {
                if(a.getState() != Action.State.DEFAULT && a.DEBUG_showChildren()) node.addChild(GetTreeFromAction(a));
            }
        return node;
    }
}