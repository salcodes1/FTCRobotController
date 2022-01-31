package org.firstinspires.ftc.teamcode.bt;

import android.util.Pair;

public interface Node<T> {
    enum Result {
        SUCCESS,
        FAIL,
        RUNNING
    }

    Pair<Result, T> exec();
}
