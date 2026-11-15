// package name
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicShooter {
    private DcMotorEx shooter;

    /**
     * Initializes the shooter motor hardware.
     * @param hardwareMap the robot hardware map from the OpMode
     */
    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets the target velocity of the shooter motor.
     * @param velocity the target velocity in encoder ticks per second
     */
    public void setVelocity(double velocity) {
        shooter.setVelocity(velocity);
    }

    /**
     * Stops the shooter motor.
     */
    public void stop() {
        shooter.setVelocity(0);
    }

    /**
     * Gets the current speed of the shooter in rotations per minute.
     * @return the current shooter speed in RPM
     */
    public double getRPM() {
        return (shooter.getVelocity() / 28.0) * 60.0;
    }
}



