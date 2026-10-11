package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class PrototypeShooter extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            shooter.setPower(1);
        }
    }
}
