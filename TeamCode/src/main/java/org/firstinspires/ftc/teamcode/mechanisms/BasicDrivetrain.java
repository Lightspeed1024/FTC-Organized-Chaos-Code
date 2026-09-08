package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.RobotMath;

public class BasicDrivetrain {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    public enum Motor {LEFT_MOTOR, RIGHT_MOTOR}
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private Gamepad gamepad1;
    private ElapsedTime runtime = new ElapsedTime();

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    // Change these measurements if the motors, wheels, or gearing change.
    private static final double COUNTS_PER_MOTOR_REV = 560.0;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;
    private static final double TRACK_WIDTH_INCHES = 16.0;
    private static final double SPEED_UP_RATE = 2.75;
    private static final double SLOW_DOWN_RATE = 5.50;
    private static final double TURN_CIRCUMFERENCE = Math.PI * TRACK_WIDTH_INCHES;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);

    public static final double DEAD_ZONE = 0.06;


    // Instance variables: these are not static or final and can be customized in each TeleOp.
    public double normalSpeed = 0.75;
    public double fastSpeed = 1.00;
    public double slowSpeed = 0.35;
    public double turnSpeed = 0.80;
    public double movingTurnSpeed = 0.55;
    public double speedUpRate = 2.75;
    public double slowDownRate = 5.50;

    // Driving Outputs (Read-Only)
    public double slowAmount;
    public double boostAmount;
    public double speedLimit;
    public double wantedLeftPower;
    public double wantedRightPower;
    public double turnLimit;

    /**
     * A replacement for the constructor of this class (making a custom method allows more functionalities than constructor).
     * It initializes all the motors and configures their settings.
     * NEEDS TO BE CALLED EVERY TIME THIS CLASS IS INSTANTIATED.
     * @param opMode Pass in "this" in the OpMode. It will give the OpMode object,
     *               allowing this class to use things like telemetry.
     * @param hardwareMap Pass in hardwareMap in the OpMode, letting this class access the maps of the motors on the control hub.
     */
    public void init(LinearOpMode opMode, HardwareMap hardwareMap) {
        telemetry = opMode.telemetry;
        gamepad1 = opMode.gamepad1;

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // The motors face opposite directions, so the right motor is reversed.
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    public void driveTeleOp(double loopTime) {

        // The y-axis of gamepads are reversed
        double drive = RobotMath.fixJoystick(gamepad1.left_stick_y, DEAD_ZONE);
        double turn = RobotMath.fixJoystick(-gamepad1.right_stick_x, DEAD_ZONE);

        // Squaring makes small stick movements easier to control.
        drive = Math.copySign(drive * drive, drive);
        turn = Math.copySign(turn * turn, turn);

        slowAmount = Range.clip(gamepad1.left_trigger, 0.0, 1.0);
        boostAmount = Range.clip(gamepad1.right_trigger, 0.0, 1.0);

        // The left trigger slows down the speed, while the right trigger boosts it up
        if (slowAmount > 0.05) {
            speedLimit = RobotMath.interpolate(normalSpeed, slowSpeed, slowAmount);
        }
        else {
            speedLimit = RobotMath.interpolate(normalSpeed, fastSpeed, boostAmount);
        }

        // Turning is less sensitive while the robot is moving quickly.
        turnLimit = RobotMath.interpolate(turnSpeed, movingTurnSpeed, Math.abs(drive));
        turn *= turnLimit;

        wantedLeftPower = drive + turn;
        wantedRightPower = drive - turn;

        // Scale both powers equally if either one is above full power.
        double biggestPower = Math.max(
                Math.abs(wantedLeftPower),
                Math.abs(wantedRightPower)
        );

        if (biggestPower > 1.0) {
            wantedLeftPower /= biggestPower;
            wantedRightPower /= biggestPower;
        }

        wantedLeftPower *= speedLimit;
        wantedRightPower *= speedLimit;

        setSmoothDrivePower(wantedLeftPower, wantedRightPower, loopTime);
    }

    /**
     * Directly sets the powers for both of the motors and updates them instantly.
     * @param leftPower Left motor power.
     * @param rightPower Right motor power.
     */
    public void setDrivePower(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }

    /**
     * Changes motor power gradually so the robot does not jerk forward or backward.
     */
    public void setSmoothDrivePower(double wantedLeftPower, double wantedRightPower, double loopTime) {
        double newLeftPower = smoothPower(leftPower, wantedLeftPower, loopTime);
        double newRightPower = smoothPower(rightPower, wantedRightPower, loopTime);

        setDrivePower(newLeftPower, newRightPower);
    }

    /**
     * Calculates the maximum change for a single motor every loop.
     */
    private double smoothPower(double currentPower, double wantedPower, double loopTime) {
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
    private double moveToward(double current, double target, double maximumChange) {
        double change = Range.clip(target - current, -maximumChange, maximumChange);
        return current + change;
    }

    /**
     * Uses the motor encoders to move each side of the robot a set distance.
     * Positive distances move forward, and negative distances move backward.
     */
    public void driveInches(double speed, double leftInches, double rightInches, double timeoutSeconds) {
        if (!opMode.opModeIsActive()) {
            return;
        }
        double drivePower = Range.clip(Math.abs(speed), 0.0, 1.0);
        if (drivePower == 0.0 || timeoutSeconds <= 0.0) {
            stop();
            return;
        }

        int leftTarget = leftMotor.getCurrentPosition() + inchesToTicks(leftInches);
        int rightTarget = rightMotor.getCurrentPosition() + inchesToTicks(rightInches);

        leftMotor.setTargetPosition(leftTarget);
        rightMotor.setTargetPosition(rightTarget);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        try {
            setDrivePower(drivePower, drivePower);

            // Wait until both motors finish, time runs out, or the OpMode stops.
            while (opMode.opModeIsActive()
                    && runtime.seconds() < timeoutSeconds
                    && (leftMotor.isBusy() || rightMotor.isBusy())) {

                telemetry.addData(
                        "Target",
                        "Left: %d  Right: %d",
                        leftTarget,
                        rightTarget
                );

                telemetry.addData(
                        "Position",
                        "Left: %d  Right: %d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition()
                );

                telemetry.addData(
                        "Time",
                        "%.1f / %.1f seconds",
                        runtime.seconds(),
                        timeoutSeconds
                );

                telemetry.update();
                opMode.idle();
            }
        } finally {
            stop();
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Turns the robot using encoder distances. Positive degrees turn clockwise.
     */
    public void turnDegrees(double speed, double degrees, double timeoutSeconds) {
        double inches = (degrees / 360.0) * TURN_CIRCUMFERENCE;
        driveInches(speed, inches, -inches, timeoutSeconds);
    }

    public double getPower(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR: return leftPower;
            case RIGHT_MOTOR: return rightPower;
            default: return 0.0;
        }
    }

    public boolean isDriving() {
        return leftMotor.isBusy() || rightMotor.isBusy();
    }

    public void resetEncoders() {
        stop();

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop() {
        leftPower = 0.0;
        rightPower = 0.0;
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    public void setMotorSpeed(Motor motor, double speed) {
        setPower(motor, speed);
    }

    public int getCurrentPosition(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR: return leftMotor.getCurrentPosition();
            case RIGHT_MOTOR: return rightMotor.getCurrentPosition();
            default: return 0;
        }
    }

    public void setTargetPosition(Motor motor, int target) {
        switch (motor) {
            case LEFT_MOTOR: leftMotor.setTargetPosition(target); break;
            case RIGHT_MOTOR: rightMotor.setTargetPosition(target); break;
        }
    }

    public void setPower(Motor motor, double power) {
        double clippedPower = Range.clip(power, -1.0, 1.0);

        switch (motor) {
            case LEFT_MOTOR:
                leftPower = clippedPower;
                leftMotor.setPower(clippedPower);
                break;

            case RIGHT_MOTOR:
                rightPower = clippedPower;
                rightMotor.setPower(clippedPower);
                break;
        }
    }

    public void setMode(Motor motor, DcMotor.RunMode mode) {
        switch (motor) {
            case LEFT_MOTOR: leftMotor.setMode(mode);break;
            case RIGHT_MOTOR: rightMotor.setMode(mode);break;
        }
    }

    public boolean isBusy(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR: return leftMotor.isBusy();
            case RIGHT_MOTOR: return rightMotor.isBusy();
            default: return false;
        }
    }

    /**
     * Converts a distance in inches into motor encoder ticks.
     */
    private int inchesToTicks(double inches) {
        return (int) Math.round(inches * COUNTS_PER_INCH);
    }
}
