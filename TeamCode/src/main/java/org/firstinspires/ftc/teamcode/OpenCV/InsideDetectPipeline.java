package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMechanism;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@Config
public class InsideDetectPipeline extends OpenCvPipeline {


    Mat mat = new Mat();
    Mat gateMatYellow = new Mat();
    Mat gateMatWhite = new Mat();
    public Rect chosenRect;

    Telemetry telemetry;

    public static MyScalar yellowLowHSV = new MyScalar(0, 70, 30);
    public static MyScalar yellowHighHSV = new MyScalar(45, 255, 255);

    public static MyScalar whiteLowHSV = new MyScalar(90, 0, 170);
    public static MyScalar whiteHighHSV = new MyScalar(255, 30, 255);

    boolean pieceDetected = false;

    public InsideDetectPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Core.normalize(input, mat, 0, 255, Core.NORM_MINMAX);


        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Imgproc.medianBlur(mat, mat, 5);

        // Yellow

        Core.inRange(mat, yellowLowHSV.toScalar(), yellowHighHSV.toScalar(), gateMatYellow);
        Imgproc.morphologyEx(gateMatYellow, gateMatYellow, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(gateMatYellow, gateMatYellow, Imgproc.MORPH_CLOSE, new Mat());

        // White

        Core.inRange(mat, whiteLowHSV.toScalar(), whiteHighHSV.toScalar(), gateMatWhite);

        Core.bitwise_or(gateMatWhite, gateMatYellow, gateMatYellow);

//        Imgproc.GaussianBlur(gateMatYellow, gateMatYellow, new Size(5, 5), 0);
        Imgproc.morphologyEx(gateMatYellow, gateMatYellow, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(gateMatYellow, gateMatYellow, Imgproc.MORPH_CLOSE, new Mat());


        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(gateMatYellow, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_TC89_KCOS);

        Imgproc.cvtColor(gateMatYellow, gateMatYellow, Imgproc.COLOR_GRAY2RGB);

        Imgproc.drawContours(gateMatYellow, contours, -1, new Scalar(0, 255, 255));

        chosenRect = null;

        for (MatOfPoint contour :
                contours) {
            Rect rect = Imgproc.boundingRect(contour);
            if(rect.size().area() > chosenRect.size().area())
                chosenRect = rect;
        }


        hierarchy.release();
        return gateMatYellow;
    }

    public boolean pieceInside() { return pieceDetected; }

}
