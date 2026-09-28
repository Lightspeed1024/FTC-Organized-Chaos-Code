package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSideWheels {

    private CRServo leftWheel;
//    private CRServo rightWheel;

    public void init(HardwareMap hardwareMap) {
        leftWheel = hardwareMap.get(CRServo.class, "intakeServo");
        leftWheel.setDirection(CRServo.Direction.FORWARD);
    }

    public void setPower(double power) {
        leftWheel.setPower(power);
    }
}
