package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {

    // FTC distance sensor
    private com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;

    /**
     * The initialization method for the distance sensor.
     * NEEDS TO BE CALLED EVERY TIME YOU INSTANTIATE THIS CLASS!!!
     * @param hwMap Pass in hardwareMap in the OpMode
     */
    public void init(HardwareMap hwMap) {
        distanceSensor = hwMap.get(
                com.qualcomm.robotcore.hardware.DistanceSensor.class,
                "distanceSensor"
        );
    }

    /**
     * Get the distance to detection in inches
     */
    public double getDistanceInches() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
}
