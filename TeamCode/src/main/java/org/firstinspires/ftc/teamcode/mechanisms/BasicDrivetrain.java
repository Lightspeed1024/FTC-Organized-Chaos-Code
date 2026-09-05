package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BasicDrivetrain {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private LinearOpMode opMode;
    private final ElapsedTime runtime = new ElapsedTime();

    public enum Motor {
        LEFT_MOTOR,
        RIGHT_MOTOR
    }

    // Motor and wheel measurements
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.54331;
    static final double TRACK_WIDTH_INCHES = 16.0;

    static final double TURN_CIRCUMFERENCE =
            Math.PI * TRACK_WIDTH_INCHES;

    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);

    public void init(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoders first
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Then allow the motors to run with the encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Drives each side of the robot a specified number of inches.
     * Positive values move forward and negative values move backward.
     */
    public void driveInches(
            double speed,
            double leftInches,
            double rightInches,
            double timeoutSeconds) {

        if (!opMode.opModeIsActive()) {
            return;
        }

        int newLeftTarget =
                leftMotor.getCurrentPosition()
                        + (int) Math.round(leftInches * COUNTS_PER_INCH);

        int newRightTarget =
                rightMotor.getCurrentPosition()
                        + (int) Math.round(rightInches * COUNTS_PER_INCH);

        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        leftMotor.setPower(Math.abs(speed));
        rightMotor.setPower(Math.abs(speed));

        // Keep going until both motors finish, time runs out,
        // or the OpMode is stopped.
        while (opMode.opModeIsActive()
                && runtime.seconds() < timeoutSeconds
                && (leftMotor.isBusy() || rightMotor.isBusy())) {

            opMode.telemetry.addData(
                    "Target",
                    "%7d : %7d",
                    newLeftTarget,
                    newRightTarget
            );

            opMode.telemetry.addData(
                    "Position",
                    "%7d : %7d",
                    leftMotor.getCurrentPosition(),
                    rightMotor.getCurrentPosition()
            );

            opMode.telemetry.update();
            opMode.idle();
        }

        stop();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Small pause between autonomous movements
        opMode.sleep(250);
    }

    /**
     * Turns the robot by a specified number of degrees.
     * Positive values turn clockwise.
     * Negative values turn counterclockwise.
     */
    public void turnDegrees(
            double speed,
            double degrees,
            double timeoutSeconds) {

        double wheelTravel =
                (degrees / 360.0) * TURN_CIRCUMFERENCE;

        driveInches(
                speed,
                wheelTravel,
                -wheelTravel,
                timeoutSeconds
        );
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void setMotorSpeed(Motor motor, double speed) {
        setPower(motor, speed);
    }

    public int getCurrentPosition(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR:
                return leftMotor.getCurrentPosition();

            case RIGHT_MOTOR:
                return rightMotor.getCurrentPosition();

            default:
                return 0;
        }
    }

    public void setTargetPosition(Motor motor, int target) {
        switch (motor) {
            case LEFT_MOTOR:
                leftMotor.setTargetPosition(target);
                break;

            case RIGHT_MOTOR:
                rightMotor.setTargetPosition(target);
                break;
        }
    }

    public void setPower(Motor motor, double power) {
        switch (motor) {
            case LEFT_MOTOR:
                leftMotor.setPower(power);
                break;

            case RIGHT_MOTOR:
                rightMotor.setPower(power);
                break;
        }
    }

    public void setMode(Motor motor, DcMotor.RunMode mode) {
        switch (motor) {
            case LEFT_MOTOR:
                leftMotor.setMode(mode);
                break;

            case RIGHT_MOTOR:
                rightMotor.setMode(mode);
                break;
        }
    }

    public boolean isBusy(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR:
                return leftMotor.isBusy();

            case RIGHT_MOTOR:
                return rightMotor.isBusy();

            default:
                return false;
        }
    }
}
