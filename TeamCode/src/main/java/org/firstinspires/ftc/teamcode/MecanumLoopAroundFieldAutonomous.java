package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.BasicDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrivetrain;

@Autonomous
public class MecanumLoopAroundFieldAutonomous extends LinearOpMode {
    MecanumDrivetrain drivetrain = new MecanumDrivetrain();
//    private final BasicDrivetrain.Motor leftMotor = BasicDrivetrain.Motor.LEFT_MOTOR;
//    private final BasicDrivetrain.Motor rightMotor = BasicDrivetrain.Motor.RIGHT_MOTOR;

    static final double     DRIVE_SPEED             = 1.0;      // Full speed ahead!
    static final double     TURN_SPEED              = 0.7;      // Slightly slower turning for precision but faster than TeleOp
    private static final double SPEED_UP_RATE = 2.75;
    private static final double SLOW_DOWN_RATE = 5.50;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this, hardwareMap, SLOW_DOWN_RATE, SPEED_UP_RATE);
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        drivetrain.driveInches(DRIVE_SPEED, 132, 0, 0, 10);
        drivetrain.driveInches(DRIVE_SPEED, 0, -132, 0, 10);
        drivetrain.driveInches(DRIVE_SPEED, -132, 0, 0, 10);
        drivetrain.driveInches(DRIVE_SPEED, 0, 132, 0, 10);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(3000);  // pause to display final telemetry message.
    }
}