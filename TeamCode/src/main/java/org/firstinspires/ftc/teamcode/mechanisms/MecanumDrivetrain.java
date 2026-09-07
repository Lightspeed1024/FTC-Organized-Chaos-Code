package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrivetrain {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    /**
     * The initializer for the MecanumDrivetrain class.
     * NEEDS TO BE CALLED EVERY TIME THIS CLASS IS INSTANTIATED.
     * @param opMode Pass in "this". It will provide the OpMode functionalities to this class.
     * @param hwMap Pass in "hardwareMap". This will give the class access to the motor configurations on the Control Hub.
     */
    public void init(LinearOpMode opMode, HardwareMap hwMap) {
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

    private MecanumPowers calculateMotorPowers(double forward, double strafe, double turn) {
        double frontLeftPower = forward + strafe + turn;
        double backLeftPower = forward - strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backRightPower = forward + strafe - turn;

        double highestPower = 1.0;

        highestPower = Math.max(highestPower, Math.abs(frontLeftPower));
        highestPower = Math.max(highestPower, Math.abs(backLeftPower));
        highestPower = Math.max(highestPower, Math.abs(frontRightPower));
        highestPower = Math.max(highestPower, Math.abs(backRightPower));

        frontLeftPower /= highestPower;
        backLeftPower /= highestPower;
        frontRightPower /= highestPower;
        backRightPower /= highestPower;

        return new MecanumPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    /**
     * A robot-relative drive method. The movements are made relative to the robot's POV.
     * @param forward The power at which to drive forward, typically given by the left stick y-axis on the gamepad.
     * @param strafe The power to move sideways, typically the x-axis of the left stick.
     * @param turn The power to rotate, typically the x-axis of the right stick.
     */
    public void drive(double forward, double strafe, double turn) {
        MecanumPowers motorPowers = calculateMotorPowers(forward, strafe, turn);

        frontLeftMotor.setPower(motorPowers.frontLeftPower);
        backLeftMotor.setPower(motorPowers.backLeftPower);
        frontRightMotor.setPower(motorPowers.frontRightPower);
        backRightMotor.setPower(motorPowers.backRightPower);
    }

    /**
     * A field-oriented version of driving, where forwards and sideways movement is relative to the driver's POV.
     * @param forward The power at which to drive forward, typically given by the left stick y-axis on the gamepad.
     * @param strafe The power to move sideways, typically the x-axis of the left stick.
     * @param turn The power to rotate at, typically the x-axis of the right stick.
     */
    public void driveFieldRelative(double forward, double strafe, double turn) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, turn);
    }

    private static class MecanumPowers {
        public double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

        public MecanumPowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
            this.frontLeftPower = frontLeftPower;
            this.backLeftPower = backLeftPower;
            this.frontRightPower = frontRightPower;
            this.backRightPower = backRightPower;
        }
    }
}
