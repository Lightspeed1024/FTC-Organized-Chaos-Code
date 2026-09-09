package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.BasicIntake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrivetrain;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    private final ElapsedTime loopTimer = new ElapsedTime();

    private static final double NORMAL_SPEED = 0.75;
    private static final double FAST_SPEED = 1.00;
    private static final double SLOW_SPEED = 0.35;
    private static final double DEAD_ZONE = 0.06;

    private final MecanumDrivetrain drivetrain = new MecanumDrivetrain();
    private final BasicIntake intake = new BasicIntake();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this, hardwareMap);
        intake.init(hardwareMap);

        telemetry.addLine("Robot is ready");
        telemetry.addLine("Left stick: drive and strafe");
        telemetry.addLine("Right stick X: turn");
        telemetry.addLine("Normal speed: 75%");
        telemetry.addLine("Right trigger: boost to 100%");
        telemetry.addLine("Left trigger: 35% slow mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            drivetrain.drive(0.0, 0.0, 0.0, 0.0, false);
            return;
        }

        loopTimer.reset();

        try {
            while (opModeIsActive()) {
                // Keep the loop time from getting too high if the code pauses.
                double loopTime = Math.min(loopTimer.seconds(), 0.10);
                loopTimer.reset();

                // The Y value is negative when the stick goes forward, so flip it.
                double forward = shapeJoystick(-gamepad1.left_stick_y);
                double strafe = shapeJoystick(gamepad1.left_stick_x);
                double turn = shapeJoystick(gamepad1.right_stick_x);

                double slowAmount = Range.clip(gamepad1.left_trigger, 0.0, 1.0);
                double boostAmount = Range.clip(gamepad1.right_trigger, 0.0, 1.0);

                double speedLimit;
                String driveMode;

                if (slowAmount > 0.05) {
                    speedLimit = interpolate(NORMAL_SPEED, SLOW_SPEED, slowAmount);
                    driveMode = "SLOW";
                } else {
                    speedLimit = interpolate(NORMAL_SPEED, FAST_SPEED, boostAmount);
                    driveMode = boostAmount > 0.05 ? "BOOST" : "NORMAL";
                }

                forward *= speedLimit;
                strafe *= speedLimit;
                turn *= speedLimit;

                // Send the stick values to the drivetrain. True turns smoothing on.
                drivetrain.drive(forward, strafe, turn, loopTime, true);

                if (gamepad2.right_trigger > 0.05) {
                    intake.spinIntake(gamepad2.right_trigger);
                } else if (gamepad2.left_trigger > 0.05) {
                    intake.spinIntake(-gamepad2.left_trigger);
                } else {
                    intake.spinIntake(0.0);
                }

                telemetry.addData("Drive Mode", driveMode);
                telemetry.addData("Speed Limit", "%.0f%%", speedLimit * 100.0);
                telemetry.addData(
                        "Drive Command",
                        "Forward: %.2f  Strafe: %.2f  Turn: %.2f",
                        forward,
                        strafe,
                        turn
                );
                telemetry.addData("Intake Speed", intake.getSpeed());
                telemetry.update();
                idle();
            }
        } finally {
            intake.spinIntake(0.0);
            drivetrain.drive(0.0, 0.0, 0.0, 0.0, false);
        }
    }

    /** Gets rid of joystick drift and makes small movements easier to control. */
    private double shapeJoystick(double stickValue) {
        double amount = Math.abs(stickValue);

        if (amount <= DEAD_ZONE) {
            return 0.0;
        }

        double fixedAmount = (amount - DEAD_ZONE) / (1.0 - DEAD_ZONE);
        return Math.copySign(fixedAmount * fixedAmount, stickValue);
    }

    private double interpolate(double start, double end, double amount) {
        amount = Range.clip(amount, 0.0, 1.0);
        return start + amount * (end - start);
    }
}
