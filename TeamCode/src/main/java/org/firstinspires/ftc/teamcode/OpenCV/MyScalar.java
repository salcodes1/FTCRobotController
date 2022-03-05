package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Scalar;

public class MyScalar {
    public int H, S, V;

    MyScalar(int H, int S, int V) {
        this.H = H;
        this.S = S;
        this.V = V;
    }
    Scalar toScalar() {
        return new Scalar(H, S, V);
    }
}
