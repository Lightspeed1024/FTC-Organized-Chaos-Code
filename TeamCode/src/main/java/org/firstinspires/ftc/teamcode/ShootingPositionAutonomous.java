package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrivetrain;

@Autonomous(name="Left Corner - Blue Hive", group="Autonomous")
public class ShootingPositionAutonomous extends LinearOpMode {

    private final MecanumDrivetrain drivetrain = new MecanumDrivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the Drivetrain
        drivetrain.init(this, hardwareMap);

        waitForStart();
        if (opModeIsActive()) {


            // Go to shooting position to hive

            //  Move forward off the wall ( 2.5 field tiles out)
            telemetry.addLine(" Driving  toward hive line");
            telemetry.update();
            drivetrain.driveInches(0.5, 60.0, 0.0, 0.0, 4.0);
            sleep(200);

            // Strafe right to line up with Blue Hive
            telemetry.addLine(" align with Blue Hive");
            telemetry.update();
            drivetrain.strafeInches(0.5, 36.0, 3.0);
            sleep(200);


            //  SCORE ELEMENTS

            telemetry.addLine("Shoot");
            telemetry.update();

            // We got to put space here for shooter code
            sleep(2000);


            // go back TO LEFT DOWN CORNER

            telemetry.addLine(" back to base left corner");
            telemetry.update();

            //  Strafe LEFT 36 inches to return to the left wall line
            drivetrain.strafeInches(0.5, -36.0, 3.0);
            sleep(200);

            // Drive BACKWARD 60 inches to return to left  corner
            drivetrain.driveInches(0.5, -60.0, 0.0, 0.0, 4.0);

            telemetry.addLine("parked!");
            telemetry.update();
        }
    }
}

//Don't' know why I added so many driver station things
