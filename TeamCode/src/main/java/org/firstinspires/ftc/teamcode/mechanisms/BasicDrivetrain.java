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

public class BasicDrivetrain extends Drivetrain{
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    public enum Motor {LEFT_MOTOR, RIGHT_MOTOR}
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    // Change these measurements if the motors, wheels, or gearing change.
    private static final double COUNTS_PER_MOTOR_REV = 560.0;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;
    private static final double TRACK_WIDTH_INCHES = 16.0;
    private static final double TURN_CIRCUMFERENCE = Math.PI * TRACK_WIDTH_INCHES;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);
    private double slowDownRate;
    private double speedUpRate;
    private double wantedLeftPower;
    private double wantedRightPower;

    /**
     * A replacement for the constructor of this class (making a custom method allows more functionalities than constructor).
     * It initializes all the motors and configures their settings.
     * NEEDS TO BE CALLED EVERY TIME THIS CLASS IS INSTANTIATED.
     * @param opMode Pass in "this" in the OpMode. It will give the OpMode object,
     *               allowing this class to use things like telemetry.
     * @param hardwareMap Pass in hardwareMap in the OpMode, letting this class access the maps of the motors on the control hub.
     */
    public void init(LinearOpMode opMode, HardwareMap hardwareMap, double slowDownRate, double speedUpRate) {
        telemetry = opMode.telemetry;
        this.slowDownRate = slowDownRate;
        this.speedUpRate = speedUpRate;


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

    public void drive(double drive, double turn, double loopTime, boolean smooth) {
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

        if (smooth) {
            setSmoothDrivePower(wantedLeftPower, wantedRightPower, loopTime);
        }
        else {
            setDrivePower(wantedLeftPower, wantedRightPower);
        }
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
    public void setSmoothDrivePower(double wantedLeftPower,
                                    double wantedRightPower,
                                    double loopTime) {
        double newLeftPower = smoothPower(leftPower, wantedLeftPower, slowDownRate, speedUpRate, loopTime);
        double newRightPower = smoothPower(rightPower, wantedRightPower, slowDownRate, speedUpRate, loopTime);

        setDrivePower(newLeftPower, newRightPower);
    }

    /**
     * Uses the motor encoders to move each side of the robot a set distance.
     * Positive distances move forward, and negative distances move backward.
     * @param speed The speed to drive at.
     * @param leftInches The distance to move the left wheels.
     * @param rightInches The distance to move the right wheels.
     * @param timeoutSeconds The amount of seconds after which to stop movement even if it is incomplete.
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
                        rightTarget);
                telemetry.addData(
                        "Position",
                        "Left: %d  Right: %d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.addData(
                        "Time",
                        "%.1f / %.1f seconds",
                        runtime.seconds(),
                        timeoutSeconds);
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

    public void stop() {
        leftPower = 0.0;
        rightPower = 0.0;
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
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

    public double getPower(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR: return leftPower;
            case RIGHT_MOTOR: return rightPower;
            default: return 0.0;
        }
    }

    public double getWantedPower(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR: return wantedLeftPower;
            case RIGHT_MOTOR: return wantedRightPower;
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
