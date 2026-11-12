package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.BasicIntake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Odometry;
import org.firstinspires.ftc.teamcode.utilities.RobotMath;

@TeleOp(name = "Mecanum TeleOp with Odometry")
public class MecanumTeleOp extends LinearOpMode {
    private final ElapsedTime loopTimer = new ElapsedTime();

    private static final double NORMAL_SPEED = 0.75;
    private static final double FAST_SPEED = 1.00;
    private static final double SLOW_SPEED = 0.35;
    private static final double DEAD_ZONE = 0.06;
    private static final double SPEED_UP_RATE = 2.75;
    private static final double SLOW_DOWN_RATE = 5.50;
    private static final double TURN_SPEED = 0.80;
    private static final double MOVING_TURN_SPEED = 0.55;

    private double speedLimit;
    private double turnLimit;

    private final MecanumDrivetrain.Motor frontLeftMotor =
            MecanumDrivetrain.Motor.FRONT_LEFT_MOTOR;
    private final MecanumDrivetrain.Motor frontRightMotor =
            MecanumDrivetrain.Motor.FRONT_RIGHT_MOTOR;
    private final MecanumDrivetrain.Motor backLeftMotor =
            MecanumDrivetrain.Motor.BACK_LEFT_MOTOR;
    private final MecanumDrivetrain.Motor backRightMotor =
            MecanumDrivetrain.Motor.BACK_RIGHT_MOTOR;

    private final MecanumDrivetrain drivetrain = new MecanumDrivetrain();
    private final Odometry odometry = new Odometry();
    private final BasicIntake intake = new BasicIntake();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this, hardwareMap, SLOW_DOWN_RATE, SPEED_UP_RATE);
        odometry.init(hardwareMap);
        intake.init(hardwareMap);

        telemetry.addLine("Robot is ready");
        telemetry.addLine("Left stick: drive and strafe");
        telemetry.addLine("Right stick X: turn");
        telemetry.addLine("Right trigger: boost");
        telemetry.addLine("Left trigger: slow mode");
        telemetry.addLine("Drive direction follows the field, using Pinpoint heading");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            drivetrain.drive(0.0, 0.0, 0.0, 0.0, false);
            return;
        }

        loopTimer.reset();

        try {
            while (opModeIsActive()) {
                double loopTime = Math.min(loopTimer.seconds(), 0.10);
                loopTimer.reset();

                double forward =
                        RobotMath.fixJoystick(-gamepad1.left_stick_y, DEAD_ZONE);
                double strafe =
                        RobotMath.fixJoystick(gamepad1.left_stick_x, DEAD_ZONE);
                double turn =
                        RobotMath.fixJoystick(gamepad1.right_stick_x, DEAD_ZONE);

                 // Squaring makes small stick movements easier to control.
                forward = Math.copySign(forward * forward, forward);
                strafe = Math.copySign(strafe * strafe, strafe);
                turn = Math.copySign(turn * turn, turn);

                double slowAmount =
                        Range.clip(gamepad1.left_trigger, 0.0, 1.0);
                double boostAmount =
                        Range.clip(gamepad1.right_trigger, 0.0, 1.0);

                String driveMode;
                if (slowAmount > 0.05) {
                    speedLimit = RobotMath.interpolate(
                            NORMAL_SPEED, SLOW_SPEED, slowAmount);
                    driveMode = "SLOW";
                } else {
                    speedLimit = RobotMath.interpolate(
                            NORMAL_SPEED, FAST_SPEED, boostAmount);
                    driveMode = boostAmount > 0.05 ? "BOOST" : "NORMAL";
                }

                forward *= speedLimit;
                strafe *= speedLimit;

                double movementAmount = Range.clip(
                        Math.hypot(forward, strafe), 0.0, 1.0);
                turnLimit = RobotMath.interpolate(
                        TURN_SPEED, MOVING_TURN_SPEED, movementAmount);
                turn *= speedLimit * turnLimit;

                odometry.update();
                drivetrain.driveFieldRelative(
                        forward,
                        strafe,
                        turn,
                        odometry.getHeadingRadians(),
                        loopTime,
                        true);

                if (gamepad2.right_trigger > 0.05) {
                    intake.spinIntake(gamepad2.right_trigger);
                } else if (gamepad2.left_trigger > 0.05) {
                    intake.spinIntake(-gamepad2.left_trigger);
                } else {
                    intake.spinIntake(0.0);
                }

                telemetry.addData("Drive Mode", driveMode);
                telemetry.addData(
                        "Speed Limit", "%.0f%%", speedLimit * 100.0);
                telemetry.addData(
                        "Odo X", "%.1f in", odometry.getXInches());
                telemetry.addData(
                        "Odo Y", "%.1f in", odometry.getYInches());
                telemetry.addData(
                        "Odo Heading", "%.1f deg", odometry.getHeadingDegrees());
                telemetry.addData(
                        "Drive Command",
                        "Forward: %.2f  Strafe: %.2f  Turn: %.2f",
                        forward,
                        strafe,
                        turn);
                telemetry.addData(
                        "Front Powers",
                        "FL: %.2f    FR: %.2f",
                        drivetrain.getPower(frontLeftMotor),
                        drivetrain.getPower(frontRightMotor));
                telemetry.addData(
                        "Back Powers",
                        " BL: %.2f    BR: %.2f",
                        drivetrain.getPower(backLeftMotor),
                        drivetrain.getPower(backRightMotor));
                telemetry.addData(
                        "Intake Speed", intake.getSpeed());
                telemetry.update();

                idle();
            }
        } finally {
            intake.spinIntake(0.0);
            drivetrain.drive(0.0, 0.0, 0.0, 0.0, false);
        }
    }
}
