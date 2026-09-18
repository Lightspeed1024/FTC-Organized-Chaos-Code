package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensor {

    // FTC color sensor
    private com.qualcomm.robotcore.hardware.ColorSensor colorSensor;

    // Initialize the color sensor
    public void init(HardwareMap hwMap) {

        colorSensor = hwMap.get(
                com.qualcomm.robotcore.hardware.ColorSensor.class,
                "colorSensor"
        );
    }

    // Get the amount of red detected
    public int getRed() {
        return colorSensor.red();
    }

    // Get the amount of green detected
    public int getGreen() {
        return colorSensor.green();
    }

    // Get the amount of blue detected
    public int getBlue() {
        return colorSensor.blue();
    }

    // Determine which color is strongest
    public String getDetectedColor() {

        int red = getRed();
        int green = getGreen();
        int blue = getBlue();

        if (red > green && red > blue) {
            return "RED";
        }

        if (green > red && green > blue) {
            return "GREEN";
        }

        if (blue > red && blue > green) {
            return "BLUE";
        }

        return "UNKNOWN";
    }
}




