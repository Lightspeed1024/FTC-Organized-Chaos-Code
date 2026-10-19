package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class PrototypeShooter extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        FtcDashboard dashboard = FtcDashboard.getInstance();

        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("Shooter Speed", shooter.getVelocity() * 60 / 28);

            dashboard.sendTelemetryPacket(packet);
            shooter.setPower(1);
            telemetry.addData("Speed", shooter.getVelocity() * 60 / 28); // convert ticks per second to RPM
            telemetry.update();
        }
    }
}
