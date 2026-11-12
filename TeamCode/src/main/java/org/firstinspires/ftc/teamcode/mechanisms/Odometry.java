package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Reads the robot's position and heading from the goBILDA Pinpoint. */
public class Odometry {
    private static final String PINPOINT_NAME = "pinpoint";

    // Measured from robot center: X pod is left; Y pod is behind.
    private static final double X_POD_OFFSET_MM = 184.8;
    private static final double Y_POD_OFFSET_MM = -184.0;

    private GoBildaPinpointDriver pinpoint;

    public void init(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setOffsets(
                X_POD_OFFSET_MM,
                Y_POD_OFFSET_MM,
                DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Keep the robot still while Pinpoint resets its position and heading.
        pinpoint.resetPosAndIMU();
    }

    /** Refresh the readings. Call this once each TeleOp loop. */
    public void update() {
        pinpoint.update();
    }

    public double getXInches() {
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getYInches() {
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getHeadingDegrees() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public double getHeadingRadians() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }
}
