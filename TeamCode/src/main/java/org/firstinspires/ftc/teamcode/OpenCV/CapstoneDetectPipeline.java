package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CapstoneDetectPipeline extends OpenCvPipeline {

    int capstoneSegment = 0;

    @Override
    public Mat processFrame(Mat input) {
        int third = input.rows() / 3;

        Range wholeHeight = new Range(0, input.cols() - 1);
        Range segment1Range = new Range(0, third - 1);
        Range segment2Range = new Range(third, 2 * third - 1);
        Range segment3Range = new Range(2 * third, 3 * third - 1);

        double segment1 = getMagentaPercent(input.submat(segment1Range, wholeHeight));
        double segment2 = getMagentaPercent(input.submat(segment2Range, wholeHeight));
        double segment3 = getMagentaPercent(input.submat(segment3Range, wholeHeight));

        if(segment1 >= segment2 && segment1 >= segment3) capstoneSegment = 1;
        else if(segment2 >= segment1 && segment2 >= segment3) capstoneSegment = 2;
        else if(segment3 >= segment1 && segment3 >= segment2) capstoneSegment = 3;

        return input;
    }

    public double getMagentaPercent(Mat input) {
        Mat gate = new Mat();

        Scalar lowHSV = new Scalar(145, 100, 50);
        Scalar highHSV = new Scalar(160, 255, 255);

        Imgproc.cvtColor(input, gate, Imgproc.COLOR_RGB2HSV);

        Core.inRange(gate, lowHSV, highHSV, gate);

        double ratio = Core.sumElems(gate).val[0] / gate.size().area() / 255;

        gate.release();

        return ratio;
    }
}
