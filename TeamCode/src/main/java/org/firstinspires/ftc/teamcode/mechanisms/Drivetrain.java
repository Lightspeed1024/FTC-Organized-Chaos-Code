package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.util.Range;

public abstract class Drivetrain {

    /**
     * Calculates the maximum change for a single motor every loop.
     */
    protected double smoothPower(double currentPower,
                                 double wantedPower,
                                 double slowDownRate,
                                 double speedUpRate,
                                 double loopTime) {

        boolean isChangingDirection = currentPower != 0.0
                && wantedPower != 0.0
                && Math.signum(currentPower) != Math.signum(wantedPower);

        // Slow the motor to zero before making it spin in the opposite direction.
        if (isChangingDirection) {
            return moveToward(currentPower, 0.0, slowDownRate * loopTime);
        }

        boolean isSlowingDown = Math.abs(wantedPower) < Math.abs(currentPower);
        double rate = isSlowingDown ? slowDownRate : speedUpRate;

        return moveToward(currentPower, wantedPower, rate * loopTime);
    }

    /**
     * Moves a value toward its target within the bounds of the maximumChange parameter.
     */
    protected double moveToward(double current, double target, double maximumChange) {
        double change = Range.clip(target - current, -maximumChange, maximumChange);
        return current + change;
    }
}
