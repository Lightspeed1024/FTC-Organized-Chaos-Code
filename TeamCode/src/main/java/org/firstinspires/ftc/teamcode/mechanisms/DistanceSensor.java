package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {

    // FTC distance sensor
    private com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;

    // Initialize the distance sensor
    public void init(HardwareMap hwMap) {

        distanceSensor = hwMap.get(
                com.qualcomm.robotcore.hardware.DistanceSensor.class,
                "distanceSensor"
        );
    }

    // Get the distance to the object in inches
    public double getDistanceInches() {

        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    // Check if an object is closer than the stopping distance
    public boolean isObjectClose(double stoppingDistance) {

        return getDistanceInches() <= stoppingDistance;
    }
}


