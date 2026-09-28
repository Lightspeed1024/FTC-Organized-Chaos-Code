package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrivetrain;

@Autonomous(name = "Mecanum Loop Around Field")
public class MecanumLoopAroundFieldAutonomous extends LinearOpMode {
    private final MecanumDrivetrain drivetrain = new MecanumDrivetrain();

    private static final double DRIVE_SPEED = 1.0;
    private static final double STRAFE_SPEED = 1.0;
    private static final double TURN_SPEED = 0.7;
    private static final double SPEED_UP_RATE = 2.75;
    private static final double SLOW_DOWN_RATE = 5.50;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this, hardwareMap, SLOW_DOWN_RATE, SPEED_UP_RATE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Drive a square around the field.
        // Positive drive distance moves forward; negative moves backward.
        // Positive strafe distance moves right; negative moves left.
        drivetrain.driveInches(DRIVE_SPEED, 132, 132, 10.0);
        drivetrain.strafeInches(STRAFE_SPEED, -132, 10.0);
        drivetrain.driveInches(DRIVE_SPEED, -132, -132, 10.0);
        drivetrain.strafeInches(STRAFE_SPEED, 132, 10.0);

        /*
         * Additional examples:
         *
         * Drive forward 24 inches:
         * drivetrain.driveInches(DRIVE_SPEED, 24, 24, 5.0);
         *
         * Drive backward 24 inches:
         * drivetrain.driveInches(DRIVE_SPEED, -24, -24, 5.0);
         *
         * Strafe right or left 24 inches:
         * drivetrain.strafeInches(STRAFE_SPEED, 24, 5.0);
         * drivetrain.strafeInches(STRAFE_SPEED, -24, 5.0);
         *
         * Turn clockwise or counterclockwise 90 degrees:
         * drivetrain.turnDegrees(TURN_SPEED, 90, 5.0);
         * drivetrain.turnDegrees(TURN_SPEED, -90, 5.0);
         *
         * Combined movement: forward 24 inches, right 12 inches,
         * and clockwise 90 degrees in one command:
         * drivetrain.driveInches(DRIVE_SPEED, 24, 12, 90, 8.0);
         */

        drivetrain.stop();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(3000);
    }
}
