package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Odometry;

@Autonomous(name = "Shooting Position Autonomous (Odometry)")
public class ShootingPositionOdometryAutonomous extends LinearOpMode {
    private static final double DRIVE_POWER = 0.45;
    private static final double TURN_POWER = 0.30;
    private static final double DRIVE_DISTANCE_INCHES = 48.0;
    private static final double LEFT_TURN_DEGREES = -90.0;
    private static final double MOVE_TIMEOUT_SECONDS = 10.0;

    private final MecanumDrivetrain drivetrain = new MecanumDrivetrain();
    private final Odometry odometry = new Odometry();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this, hardwareMap, 5.50, 2.75);
        odometry.init(hardwareMap);

        telemetry.addLine("Odometry autonomous ready");
        telemetry.addLine("Keep the robot still until you press Start");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        // Turn left in place.
        if (!moveTo(0.0, 0.0, LEFT_TURN_DEGREES)) {
            return;
        }

        // Drive straight ahead while facing left.
        // Right is negative here because the robot is facing left of its start direction.
        if (!moveTo(0.0, -DRIVE_DISTANCE_INCHES, LEFT_TURN_DEGREES)) {
            return;
        }

        // Drive backward along the same path to the turning spot.
        if (!moveTo(0.0, 0.0, LEFT_TURN_DEGREES)) {
            return;
        }

        // Turn right to face the starting direction again.
        if (!moveTo(0.0, 0.0, 0.0)) {
            return;
        }

        drivetrain.stop();
        telemetry.addLine("Path complete");
        telemetry.update();
        sleep(1000);
    }

    private boolean moveTo(
            double forwardInches,
            double rightInches,
            double headingDegrees) {

        boolean reachedTarget = drivetrain.driveToPosition(odometry, forwardInches, rightInches, headingDegrees, DRIVE_POWER, TURN_POWER, MOVE_TIMEOUT_SECONDS);

        if (!reachedTarget) {
            telemetry.addLine("Move timed out or was stopped");
            telemetry.update();
        }

        return reachedTarget;
    }
}
