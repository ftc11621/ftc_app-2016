package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by HeavenDog on 12/1/2016.
 */

public class ColorSense {
    ColorSensor colorSensor;

    public ColorSense(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        colorSensor.enableLed(true);

    }

    public static enum Color {red, blue, neither};


    public Color senseColor() {
        if (colorSensor.red() > 200) {
            return Color.red;
        } else if (colorSensor.blue() > 200) {
            return Color.blue;
        } else {
            return Color.neither;
        }
    }
}








