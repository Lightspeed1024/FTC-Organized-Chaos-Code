package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrivetrain extends Drivetrain{
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    public enum Motor {FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR, BACK_LEFT_MOTOR, BACK_RIGHT_MOTOR}

    private static final double COUNTS_PER_MOTOR_REV = 560.0;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;
    private static final double TRACK_WIDTH_INCHES = 16.0;
    private static final double TURN_CIRCUMFERENCE = Math.PI * TRACK_WIDTH_INCHES;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);
    private ElapsedTime runtime = new ElapsedTime();

    private double frontLeftPower = 0.0;
    private double backLeftPower = 0.0;
    private double frontRightPower = 0.0;
    private double backRightPower = 0.0;
    private double speedUpRate;
    private double slowDownRate;
    private double wantedFrontLeftPower;
    private double wantedBackLeftPower;
    private double wantedFrontRightPower;
    private double wantedBackRightPower;


    /**
     * The initializer for the MecanumDrivetrain class.
     * NEEDS TO BE CALLED EVERY TIME THIS CLASS IS INSTANTIATED.
     * @param opMode Pass in "this". It will provide the OpMode functionalities to this class.
     * @param hwMap Pass in "hardwareMap". This will give the class access to the motor configurations on the Control Hub.
     */
    public void init(LinearOpMode opMode, HardwareMap hwMap, double slowDownRate, double speedUpRate) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.slowDownRate = slowDownRate;
        this.speedUpRate = speedUpRate;

        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    /**
     * A robot-relative drive method. The movements are made relative to the robot's POV.
     * @param forward The power at which to drive forward, typically given by the left stick y-axis on the gamepad.
     * @param strafe The power to move sideways, typically the x-axis of the left stick.
     * @param turn The power to rotate, typically the x-axis of the right stick.
     */
    public void driveTeleOp(double forward, double strafe, double turn, double loopTime, boolean smooth) {
        wantedFrontLeftPower = forward + strafe + turn;
        wantedBackLeftPower = forward - strafe + turn;
        wantedFrontRightPower = forward - strafe - turn;
        wantedBackRightPower = forward + strafe - turn;

        double highestPower = 1.0;

        highestPower = Math.max(highestPower, Math.abs(wantedFrontLeftPower));
        highestPower = Math.max(highestPower, Math.abs(wantedBackLeftPower));
        highestPower = Math.max(highestPower, Math.abs(wantedFrontRightPower));
        highestPower = Math.max(highestPower, Math.abs(wantedBackRightPower));

        wantedFrontLeftPower /= highestPower;
        wantedBackLeftPower /= highestPower;
        wantedFrontRightPower /= highestPower;
        wantedBackRightPower /= highestPower;

        double fl, bl, fr, br;

        if (smooth) {
            fl = smoothPower(frontLeftPower, wantedFrontLeftPower, slowDownRate, speedUpRate, loopTime);
            bl = smoothPower(backLeftPower, wantedBackLeftPower, slowDownRate, speedUpRate, loopTime);
            fr = smoothPower(frontRightPower, wantedFrontRightPower, slowDownRate, speedUpRate, loopTime);
            br = smoothPower(backRightPower, wantedBackRightPower, slowDownRate, speedUpRate, loopTime);
        }
        else {
            fl = wantedFrontLeftPower;
            bl = wantedBackLeftPower;
            fr = wantedFrontRightPower;
            br = wantedBackRightPower;
        }

        setMotorPowers(fl, bl, fr, br);
    }

    /**
     * An autonomous function to drive the robot a certain distance forward, sideways, and rotating.
     * @param speed The speed to drive at.
     * @param forward The distance in inches to drive forward.
     * @param strafe The distance in inches to move sideways: positive is right, negative is left.
     * @param turnDegrees The degrees to turn the robot: positive is clockwise, negative is counter-clockwise.
     * @param timeoutSeconds The time in seconds after which to stop movement, even if the action is unfinished.
     */
    public void driveInches(double speed, double forward, double strafe, double turnDegrees, double timeoutSeconds) {
        if (!opMode.opModeIsActive()) {
            return;
        }

        double drivePower = Range.clip(Math.abs(speed), 0.0, 1.0);

        if (drivePower == 0.0 || timeoutSeconds <= 0.0) {
            stop();
            return;
        }

        int forwardTicks = inchesToTicks(forward);
        int strafeTicks = inchesToTicks(strafe);
        int turnTicks = inchesToTicks(turnDegrees);

        int frontLeftTarget = frontLeftMotor.getCurrentPosition() + forwardTicks + strafeTicks + turnTicks;
        int frontRightTarget = frontRightMotor.getCurrentPosition() + forwardTicks - strafeTicks - turnTicks;
        int backLeftTarget = backLeftMotor.getCurrentPosition() + forwardTicks - strafeTicks + turnTicks;
        int backRightTarget = backRightMotor.getCurrentPosition() + forwardTicks + strafeTicks - turnTicks;

        frontLeftMotor.setTargetPosition(frontLeftTarget);
        frontRightMotor.setTargetPosition(frontRightTarget);
        backLeftMotor.setTargetPosition(backLeftTarget);
        backRightMotor.setTargetPosition(backRightTarget);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        try {
            setMotorPowers(drivePower, drivePower, drivePower, drivePower);

            // Wait until both motors finish, time runs out, or the OpMode stops.
            while (opMode.opModeIsActive()
                    && runtime.seconds() < timeoutSeconds
                    // TODO: change this so that instead of only one motor finishing it is 2 or 3 of them is enough
                    && (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

                telemetry.addData("Front Powers",
                        "FL: %.2f    FR: %.2f",
                        frontLeftPower,
                        frontRightPower);
                telemetry.addLine("");
                telemetry.addData("Back Powers",
                        " BL: %.2f    BR: %.2f", // DO NOT REMOVE THE SPACE IN FRONT OF BL, AS IT IS INTENDED TO ALIGN NUMBERS.
                        backLeftPower,
                        backRightPower);
                telemetry.addLine("--------------------------------");
                telemetry.addData("Front Targets",
                        "FL: %d    FR: %d",
                        frontLeftTarget,
                        frontRightTarget);
                telemetry.addData(
                        "Back Targets",
                        " BL: %d    BR: %d", // DO NOT REMOVE THE SPACE IN FRONT OF BL, AS IT IS INTENDED TO ALIGN NUMBERS.
                        backLeftTarget,
                        backRightTarget);
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
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void setMotorPowers(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);

        frontLeftPower = fl;
        backLeftPower = bl;
        frontRightPower = fr;
        backRightPower = br;
    }

    /**
     * A field-oriented version of driving, where forwards and sideways movement is relative to the driver's POV.
     * @param forward The power at which to drive forward, typically given by the left stick y-axis on the gamepad.
     * @param strafe The power to move sideways, typically the x-axis of the left stick.
     * @param turn The power to rotate at, typically the x-axis of the right stick.
     */
    public void driveFieldRelative(double forward, double strafe, double turn, double loopTime, boolean smooth) {
        double theta = Math.atan2(forward, strafe);
        double speed = Math.hypot(strafe, forward);

        double robotTheta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = speed * Math.sin(robotTheta);
        double newStrafe = speed * Math.cos(robotTheta);

        this.driveTeleOp(newForward, newStrafe, turn, loopTime, smooth);
    }

    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getCurrentPosition(Motor motor) {
        switch (motor) {
            case FRONT_LEFT_MOTOR: return frontLeftMotor.getCurrentPosition();
            case FRONT_RIGHT_MOTOR: return frontRightMotor.getCurrentPosition();
            case BACK_LEFT_MOTOR: return backLeftMotor.getCurrentPosition();
            case BACK_RIGHT_MOTOR: return backRightMotor.getCurrentPosition();
            default: return 0.0;
        }
    }

    public double getPower(Motor motor) {
        switch (motor) {
            case FRONT_LEFT_MOTOR: return frontLeftPower;
            case FRONT_RIGHT_MOTOR: return frontRightPower;
            case BACK_LEFT_MOTOR: return backLeftPower;
            case BACK_RIGHT_MOTOR: return backRightPower;
            default: return 0;
        }
    }

    public double getWantedPower(Motor motor) {
        switch (motor) {
            case FRONT_LEFT_MOTOR: return wantedFrontLeftPower;
            case FRONT_RIGHT_MOTOR: return wantedFrontRightPower;
            case BACK_LEFT_MOTOR: return wantedBackLeftPower;
            case BACK_RIGHT_MOTOR: return wantedBackRightPower;
            default: return 0;
        }
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    /**
     * Converts a distance in inches into motor encoder ticks.
     */
    private int inchesToTicks(double inches) {
        return (int) Math.round(inches * COUNTS_PER_INCH);
    }
}
