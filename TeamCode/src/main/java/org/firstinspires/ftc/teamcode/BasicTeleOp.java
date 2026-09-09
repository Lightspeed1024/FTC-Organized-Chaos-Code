package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.BasicDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.BasicIntake;
import org.firstinspires.ftc.teamcode.utilities.RobotMath;

@TeleOp
public class BasicTeleOp extends LinearOpMode {
    private final ElapsedTime loopTimer = new ElapsedTime();

    BasicDrivetrain drivetrain = new BasicDrivetrain();
    BasicIntake intake = new BasicIntake();
    public final double DEAD_ZONE = 0.06;
    private static final double NORMAL_SPEED = 0.75;
    private static final double FAST_SPEED = 1.00;
    private static final double SLOW_SPEED = 0.35;
    private static final double TURN_SPEED = 0.80;
    private static final double MOVING_TURN_SPEED = 0.55;
    private static final double SPEED_UP_RATE = 2.75;
    private static final double SLOW_DOWN_RATE = 5.50;
    public double speedLimit;
    public double turnLimit;


    private final BasicDrivetrain.Motor leftMotor = BasicDrivetrain.Motor.LEFT_MOTOR;
    private final BasicDrivetrain.Motor rightMotor = BasicDrivetrain.Motor.RIGHT_MOTOR;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this, hardwareMap);
        intake.init(hardwareMap);

        telemetry.addLine("Robot is ready");
        telemetry.addLine("Left stick: drive");
        telemetry.addLine("Right stick: turn");
        telemetry.addLine("Normal speed: 75%");
        telemetry.addLine("Right trigger: boost to 100%");
        telemetry.addLine("Left bumper: 35% slow mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            drivetrain.stop();
            return;
        }

        loopTimer.reset();

        try {
            while (opModeIsActive()) {
                double loopTime = Math.min(loopTimer.seconds(), 0.10);
                loopTimer.reset();


                // The y-axis of gamepads are reversed
                double drive = RobotMath.fixJoystick(gamepad1.left_stick_y, DEAD_ZONE);
                double turn = RobotMath.fixJoystick(gamepad1.right_stick_x, DEAD_ZONE);

                // Squaring makes small stick movements easier to control.
                drive = Math.copySign(drive * drive, drive);
                drive *= speedLimit;
                turn = Math.copySign(turn * turn, turn);

                double slowAmount = Range.clip(gamepad1.left_trigger, 0.0, 1.0);
                double boostAmount = Range.clip(gamepad1.right_trigger, 0.0, 1.0);

                // The left trigger slows down the speed, while the right trigger boosts it up
                if (slowAmount > 0.05) {
                    speedLimit = RobotMath.interpolate(NORMAL_SPEED, SLOW_SPEED, slowAmount);
                }
                else {
                    speedLimit = RobotMath.interpolate(NORMAL_SPEED, FAST_SPEED, boostAmount);
                }

                // Turning is less sensitive while the robot is moving quickly.
                turnLimit = RobotMath.interpolate(TURN_SPEED, MOVING_TURN_SPEED, Math.abs(drive));
                turn *= turnLimit;

                drivetrain.driveTeleOp(drive, turn, SLOW_DOWN_RATE, SPEED_UP_RATE, loopTime);

                if (gamepad2.right_trigger > 0.05) {
                    intake.spinIntake(gamepad2.right_trigger);
                }
                else if (gamepad2.left_trigger > 0.05) {
                    intake.spinIntake(-gamepad2.left_trigger);
                }
                else {
                    intake.spinIntake(0.0);
                }

                String driveMode;

                if (slowAmount > 0.05) {
                    driveMode = "SLOW";
                }
                else if (boostAmount > 0.05) {
                    driveMode = "BOOST";
                }
                else {
                    driveMode = "NORMAL";
                }

                telemetry.addData("Drive Mode", driveMode);
                telemetry.addData("Speed Limit", "%.0f%%", speedLimit * 100.0);
                telemetry.addData("Right Trigger", "%.0f%%", boostAmount * 100.0);
                telemetry.addData(
                        "Encoders",
                        "Left: %d  Right: %d",
                        drivetrain.getCurrentPosition(leftMotor),
                        drivetrain.getCurrentPosition(rightMotor)
                );
                telemetry.addData(
                        "Motor Power",
                        "Left: %.2f  Right: %.2f",
                        drivetrain.getPower(leftMotor),
                        drivetrain.getPower(rightMotor)
                );
                telemetry.addData("Wanted Power", "Left %.2f Right: %.2f",
                        drivetrain.wantedLeftPower, drivetrain.wantedRightPower);
                telemetry.addData("Intake Speed", intake.getSpeed());
                telemetry.update();
                idle();
            }
        } finally {
            // Always stop the motors when TeleOp ends.
            drivetrain.stop();
        }
    }
}
