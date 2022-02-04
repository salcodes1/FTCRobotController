package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CapstoneDetectPipeline extends OpenCvPipeline {

    public int capstoneSegment = 0;

    Mat viewport = new Mat();


    @Override
    public Mat processFrame(Mat input) {
        int third = input.cols() / 3;

        Range wholeHeight = new Range(0, input.rows() - 1);
        Range segment1Range = new Range(0, third - 1);
        Range segment2Range = new Range(third, 2 * third - 1);
        Range segment3Range = new Range(2 * third, 3 * third - 1);

        double segment1 = getMagentaPercent(input.submat(wholeHeight, segment1Range));
        double segment2 = getMagentaPercent(input.submat(wholeHeight, segment2Range));
        double segment3 = getMagentaPercent(input.submat(wholeHeight, segment3Range));

        if(segment1 >= segment2 && segment1 >= segment3) capstoneSegment = 1;
        else if(segment2 >= segment1 && segment2 >= segment3) capstoneSegment = 2;
        else if(segment3 >= segment1 && segment3 >= segment2) capstoneSegment = 3;


        getMagentaThresholdImage(input, viewport);

        return viewport;
    }

    public double getMagentaPercent(Mat input) {
        Mat gate = new Mat();

        getMagentaThresholdImage(input, gate);


        double ratio = Core.sumElems(gate).val[0] / gate.size().area() / 255;

        gate.release();
        return ratio;
    }

    public void getMagentaThresholdImage(Mat input, Mat out) {
        Scalar lowHSV = new Scalar(145, 100, 50);
        Scalar highHSV = new Scalar(165, 255, 255);

        Mat temp = new Mat();

        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);

        Core.inRange(temp, lowHSV, highHSV, out);
    }
}
