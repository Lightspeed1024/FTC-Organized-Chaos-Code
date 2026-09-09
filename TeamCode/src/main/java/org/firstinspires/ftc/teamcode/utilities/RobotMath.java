package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.Range;

public class RobotMath {

    /**
     * Removes small values caused by joystick drift while keeping the full range.
     */
    public static double fixJoystick(double stickValue, double deadZone) {
        double amount = Math.abs(stickValue);

        if (amount <= deadZone) {
            return 0.0;
        }

        double fixedAmount = (amount - deadZone) / (1.0 - deadZone);
        return Math.copySign(fixedAmount, stickValue);
    }

    /**
     * A linear interpolation method that moves the start value towards the end value by a certain percent.
     * For example, a start value of 10 and an end value of 30 along with an amount of 0.75 would
     * add 75% of the difference (20) to 10, returning 25.
     * @param start The start value
     * @param end The end value
     * @param amount The percent to move from start to end, WRITTEN AS A DECIMAL
     * @return The new value
     */
    public static double interpolate(double start, double end, double amount) {
        amount = Range.clip(amount, 0.0, 1.0);
        return start + amount*(end - start);
    }
}
