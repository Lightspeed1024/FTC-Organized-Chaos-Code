package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class PrototypeShooter extends LinearOpMode {

    DcMotorEx shooter;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        dashboard = FtcDashboard.getInstance();

        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            double shooterRPM = shooter.getVelocity() / 28 * 60;
            packet.put("Shooter Speed", shooterRPM);

            dashboard.sendTelemetryPacket(packet);
            shooter.setPower(1);
            telemetry.addData("Speed", shooterRPM);
            telemetry.update();
        }
    }
}
