package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMechanism;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


@Config
public class InsideDetectPipeline extends OpenCvPipeline {


    Mat mat = new Mat();
    Mat gateMatYellow = new Mat();
    Mat gateMatWhite = new Mat();

    Telemetry telemetry;


    boolean pieceDetected = false;

    public InsideDetectPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Core.normalize(input, mat, 0, 255, Core.NORM_MINMAX);


        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(mat, mat, new Size(5, 5));

        // Yellow
        Scalar lowHSV = new Scalar(0, 150, 120);
        Scalar highHSV = new Scalar(45, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, gateMatYellow);

        // White
        lowHSV = new Scalar(0, 0, 230);
        highHSV = new Scalar(255, 50, 255);

        Core.inRange(mat, lowHSV, highHSV, gateMatWhite);

            Core.bitwise_or(gateMatWhite, gateMatYellow, gateMatYellow);

        double ratio = Core.sumElems(gateMatYellow).val[0] / gateMatYellow.size().area() / 255;

        pieceDetected = ratio >= IntakeMechanism.PIXELS_RATIO;


        Imgproc.cvtColor(gateMatYellow, gateMatYellow, Imgproc.COLOR_GRAY2RGB);


        return gateMatYellow;
    }

    public boolean pieceInside() { return pieceDetected; }

}
