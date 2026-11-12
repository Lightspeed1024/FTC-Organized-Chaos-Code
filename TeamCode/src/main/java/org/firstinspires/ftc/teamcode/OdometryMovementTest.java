package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Odometry;

@Autonomous(name = "Odometry Movement Test")
public class OdometryMovementTest extends LinearOpMode {
    private static final double SLOW_DRIVE_POWER = 0.45;
    private static final double SLOW_TURN_POWER = 0.30;
    private static final double MOVE_TIMEOUT_SECONDS = 5.0;

    private final MecanumDrivetrain drivetrain = new MecanumDrivetrain();
    private final Odometry odometry = new Odometry();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this, hardwareMap, 5.50, 2.75);
        odometry.init(hardwareMap);

        telemetry.addLine("Odometry test ready");
        telemetry.addLine("Keep the robot still until you press Start");
        telemetry.update();
        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        // Go forward 12 inches, then return to the starting point.
        if (!moveTo(12.0, 0.0, 0.0)) return;
        if (!moveTo(0.0, 0.0, 0.0)) return;

        // Strafe right 12 inches, then return sideways to the start.
        if (!moveTo(0.0, 12.0, 0.0)) return;
        if (!moveTo(0.0, 0.0, 0.0)) return;

        // Turn clockwise 90 degrees, then turn back to the starting heading.
        if (!moveTo(0.0, 0.0, 90.0)) return;
        if (!moveTo(0.0, 0.0, 0.0)) return;

        drivetrain.stop();
        telemetry.addLine("Test complete");
        telemetry.update();
    }

    private boolean moveTo(double forwardInches, double rightInches, double headingDegrees) {
        boolean reached = drivetrain.driveToPosition(
                odometry,
                forwardInches,
                rightInches,
                headingDegrees,
                SLOW_DRIVE_POWER,
                SLOW_TURN_POWER,
                MOVE_TIMEOUT_SECONDS);

        if (!reached) {
            telemetry.addLine("Move timed out or OpMode stopped. Test ended.");
            telemetry.update();
        }
        return reached;
    }
}
