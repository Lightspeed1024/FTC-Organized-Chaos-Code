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
//    private static double normalSpeed = 0.75;
//    private static double fastSpeed = 1.00;
//    private static double slowSpeed = 0.35;
//    private static double deadZone = 0.06;
//    private static double turnSpeed = 0.80;
//    private static double movingTurnSpeed = 0.55;
    BasicDrivetrain drivetrain = new BasicDrivetrain();
    BasicIntake intake = new BasicIntake();

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

                drivetrain.driveTeleOp(gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.left_trigger,
                        gamepad1.right_trigger,
                        loopTime);

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

                if (drivetrain.slowAmount > 0.05) {
                    driveMode = "SLOW";
                }
                else if (drivetrain.boostAmount > 0.05) {
                    driveMode = "BOOST";
                }
                else {
                    driveMode = "NORMAL";
                }

                telemetry.addData("Drive Mode", driveMode);
                telemetry.addData("Speed Limit", "%.0f%%", drivetrain.speedLimit * 100.0);
                telemetry.addData("Right Trigger", "%.0f%%", drivetrain.boostAmount * 100.0);
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
