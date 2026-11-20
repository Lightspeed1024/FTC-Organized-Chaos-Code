// package name
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class BasicShooter {
    private DcMotorEx shooter;
    private static final double kP = 35;
    private static final double kF = 15;

    /**
     * Initializes the shooter. NEEDS TO BE CALLED EVERY TIME THIS CLASS IS INSTANTIATED!!!
     * @param hardwareMap Pass in hardwareMap in the OpMode.
     */
    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(kP, 0, 0, kF);
    }

    /**
     * Sets the target velocity of the shooter in RPM.
     */
    public void setRPM(double RPM) {
        shooter.setVelocity(RPM * 28.0 / 60);
    }

    /**
     * Stops the shooter motor. It uses the PIDF to actively brake.
     */
    public void stop() {
        shooter.setVelocity(0);
    }

    /**
     * Returns the current speed of the shooter in RPM.
     * @return the current shooter speed in RPM
     */
    public double getRPM() {
        return (shooter.getVelocity() / 28.0) * 60.0;
    }
}



