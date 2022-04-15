package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class FreightSensor {

    static double MAX_FREIGHT_DISTANCE = 50;

    Rev2mDistanceSensor distanceSensor;
    boolean detectingFreight = false;

    public FreightSensor(HardwareMap hardwareMap) {
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
    }

    public void update() {
        detectingFreight = distanceSensor.getDistance(DistanceUnit.MM) < MAX_FREIGHT_DISTANCE;
    }

    public boolean isDetectingFreight() { return detectingFreight; }
}
