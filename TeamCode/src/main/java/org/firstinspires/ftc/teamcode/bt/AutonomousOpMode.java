package org.firstinspires.ftc.teamcode.bt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.OpenCV.CapstoneDetectPipeline;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.statics.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;

import java.io.StringWriter;

import hu.webarticum.treeprinter.ListingTreePrinter;
import hu.webarticum.treeprinter.SimpleTreeNode;

public abstract class AutonomousOpMode extends LinearOpMode {

    Action rootAction;

    public SampleMecanumDrive drive;
    public DcMotor carouselMotor;
    public Outtake outtake;
    public Intake intake;
    public Servo containerServo;


    public CapstoneDetectPipeline capstoneDetection;
    public WebcamName webcamName;
    public OpenCvCamera camera;

    public Outtake.Level preloadLevel = Outtake.Level.high;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        carouselMotor = hardwareMap.get(DcMotor.class, "motorCarousel");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtake = new Outtake(this);
        intake = new Intake(this);

        containerServo = hardwareMap.get(Servo.class, "servoOuttake");






        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setAutoClear(true);

        telemetry.addLine("Precompiling trajectories;\n!DO NOT STOP THE PROCESS!");
        telemetry.update();
        precompileTrajectories();

        telemetry.addLine("Done precompiling");
        telemetry.update();

        initStart();

        while(!isStarted()) {
            initTick();
            telemetry.update();
        }

        drive.setPoseEstimate(PoseStorage.poseEstimate);
        rootAction = getRoutine();
        rootAction.start(this);

        while(opModeIsActive() && !isStopRequested()) {
            if(rootAction.hasFinished(this)) {
                rootAction.end(this);
                break;
            } else rootAction.tick(this);
            updateRobotState();
            printActionTree();
            telemetry.update();
        }

    }


    private void printActionTree() {
        StringWriter w = new StringWriter();
        SimpleTreeNode root = Utils.GetTreeFromAction(rootAction, this);
        new ListingTreePrinter().print(root, w);

        TelemetryPacket packet = new TelemetryPacket();
        String[] lines = w.toString().split("\n");

        for(String line : lines) {
            packet.addLine(line);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    void updateRobotState() {
        drive.update();
        outtake.update();

    }

    protected abstract void precompileTrajectories();

    protected abstract void initStart();

    protected abstract void initTick();

    protected abstract Action getRoutine();
}

class Utils {
    static SimpleTreeNode GetTreeFromAction(Action action, AutonomousOpMode context) {

        String[] Coresp = new String[] { "○", "◔", "◕", "●" };

        String display = action.getCustomDisplay(context).isEmpty()?
            Coresp[action.getState().ordinal()] : action.getCustomDisplay(context);

        SimpleTreeNode node = new SimpleTreeNode(
            action.getClass().getSimpleName() + " [" + display + "]"
        );
        if(action.getChildActions() != null)
            for(Action a : action.getChildActions()) {
                if(a.getState() != Action.State.DEFAULT && a.DEBUG_showChildren()) node.addChild(GetTreeFromAction(a, context));
            }
        return node;
    }
}