package org.firstinspires.ftc.teamcode.mechanisms;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensor {

    // Explicitly typed color choices to prevent string typos
    public enum DetectedColor {
        RED,
        YELLOW,
        GREEN,
        BLUE,
        UNKNOWN
    }

    private com.qualcomm.robotcore.hardware.ColorSensor colorSensor;

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensor");
    }

    public int getRed() { return colorSensor.red(); }
    public int getGreen() { return colorSensor.green(); }
    public int getBlue() { return colorSensor.blue(); }

    /**
     * Converts raw RGB values into Android's HSV float array.
     * @return float array containing [Hue (0-360), Saturation (0-1), Value (0-1)]
     */
    public float[] getHSV() {
        float[] hsv = new float[3];
        Color.RGBToHSV(getRed(), getGreen(), getBlue(), hsv);
        return hsv;
    }

    /**
     * Determines the strongest color using reliable HSV threshold angles.
     */
    public DetectedColor getDetectedColor() {
        float[] hsv = getHSV();
        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        // Filter out environments where lighting is too dark or washed out
        if (value < 0.1 || saturation < 0.2) {
            return DetectedColor.UNKNOWN;
        }

        // Categorize by Hue circle degrees (0 - 360)
        if (hue >= 340 || hue <= 20) {
            return DetectedColor.RED;
        } else if (hue >= 45 && hue <= 85) {
            return DetectedColor.YELLOW;
        } else if (hue >= 90 && hue <= 160) {
            return DetectedColor.GREEN;
        } else if (hue >= 180 && hue <= 250) {
            return DetectedColor.BLUE;
        }

        return DetectedColor.UNKNOWN;
    }
}




