package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrivetrain extends Drivetrain{
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    public enum Motor {FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR, BACK_LEFT_MOTOR, BACK_RIGHT_MOTOR}

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
        this.telemetry = opMode.telemetry;
        this.slowDownRate = slowDownRate;
        this.speedUpRate = speedUpRate;

        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

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
    public void drive(double forward, double strafe, double turn, double loopTime, boolean smooth) {
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

        if (smooth) {
            frontLeftPower = smoothPower(frontLeftPower, wantedFrontLeftPower, slowDownRate, speedUpRate, loopTime);
            backLeftPower = smoothPower(backLeftPower, wantedBackLeftPower, slowDownRate, speedUpRate, loopTime);
            frontRightPower = smoothPower(frontRightPower, wantedFrontRightPower, slowDownRate, speedUpRate, loopTime);
            backRightPower = smoothPower(backRightPower, wantedBackRightPower, slowDownRate, speedUpRate, loopTime);
        }
        else {
            frontLeftPower = wantedFrontLeftPower;
            backLeftPower = wantedBackLeftPower;
            frontRightPower = wantedFrontRightPower;
            backRightPower = wantedBackRightPower;
        }

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
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

        this.drive(newForward, newStrafe, turn, loopTime, smooth);
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
}
