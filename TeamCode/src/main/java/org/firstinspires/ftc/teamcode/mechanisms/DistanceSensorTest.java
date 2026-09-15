package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.DistanceSensor;

@TeleOp(name = "Distance Sensor Test", group = "Sensor Test")
public class DistanceSensorTest extends OpMode {

    private DistanceSensor distanceSensor = new DistanceSensor();

    @Override
    public void init() {

        // Initialize the distance sensor
        distanceSensor.init(hardwareMap);

        telemetry.addLine("Distance Sensor Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Get the distance from the sensor
        double distance = distanceSensor.getDistanceInches();

        // Show the distance on the Driver Station
        telemetry.addData(
                "Distance",
                "%.2f inches",
                distance
        );

        // Tell us if an object is within 8 inches
        if (distanceSensor.isObjectClose(8.0)) {
            telemetry.addLine("OBJECT IS CLOSE!");
        } else {
            telemetry.addLine("Object is not close");
        }

        telemetry.update();
    }
}


