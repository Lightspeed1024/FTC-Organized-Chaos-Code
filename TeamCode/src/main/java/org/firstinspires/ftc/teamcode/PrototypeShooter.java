package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous
public class PrototypeShooter extends LinearOpMode {

    DcMotorEx shooter;
    FtcDashboard dashboard;

    /**
     * The P (Proportional) coefficient for the PIDF values
     */
    double kP = 0;

    /**
     * The F (Feedforward) coefficient for the PIDF values
     */
    double kF = 0;

    double highVelocity = 1500;
    double lowVelocity = 900;
    double currTargetVelocity = highVelocity;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;
    boolean isTuning = false;
    boolean useCustomPIDF = true;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        dashboard = FtcDashboard.getInstance();

        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients defaultPIDF = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients customPIDF = new PIDFCoefficients(kP, 0, 0, kF);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.bWasPressed()) {
                // Right button (circle)
                isTuning = true;
            }
            if (gamepad1.xWasPressed()) {
                // Left button (square)
                isTuning = false;
            }

            if (isTuning) {
                if (gamepad1.leftBumperWasPressed()) {
                    useCustomPIDF = false;
                }
                if (gamepad1.rightBumperWasPressed()) {
                    useCustomPIDF = true;
                }

                if (gamepad1.aWasPressed()) {
                    // Bottom button (cross)
                    if (currTargetVelocity == highVelocity) {currTargetVelocity = lowVelocity;}
                    else { currTargetVelocity = highVelocity;}
                }

                if (gamepad1.yWasPressed()) {
                    // Top button (triangle)
                    stepIndex = (stepIndex + 1) % stepSizes.length;
                }

                if (gamepad1.dpadLeftWasPressed()) {
                    kF -= stepSizes[stepIndex];
                }
                if (gamepad1.dpadRightWasPressed()) {
                    kF += stepSizes[stepIndex];
                }

                if (gamepad1.dpadDownWasPressed()) {
                    kP += stepSizes[stepIndex];
                }
                if (gamepad1.dpadUpWasPressed()) {
                    kP -= stepSizes[stepIndex];
                }

                customPIDF.p = kP;
                customPIDF.f = kF;
                if (useCustomPIDF) {
                    shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, customPIDF);
                } else {
                    shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, defaultPIDF);
                }
                shooter.setVelocity(currTargetVelocity);
            } else {
                shooter.setPower(1);
            }

            TelemetryPacket packet = new TelemetryPacket();

            double shooterRPM = shooter.getVelocity() / 28 * 60;
            packet.put("Shooter Speed", shooterRPM);

            dashboard.sendTelemetryPacket(packet);
            telemetry.addLine("Circle to turn on tuning, square to turn off");
            telemetry.addLine("Cross to change target velocity, triangle to change step size");
            telemetry.addData("Speed", shooterRPM);
            telemetry.addData("Target Velocity", currTargetVelocity);
            telemetry.addData("Step Size", stepSizes[stepIndex]);
            telemetry.addData("P", kP);
            telemetry.addData("F", kF);
            telemetry.addData("Is Tuning", isTuning);
            telemetry.update();
        }
    }
}
