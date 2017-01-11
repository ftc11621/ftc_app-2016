package org.firstinspires.ftc.teamcode.core;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by HeavenDog on 12/1/2016.
 */

public class ColorSense {
    ColorSensor colorSensor;
    float hsvValues[] = {0F,0F,0F};
    private int redValue;
    private int greenValue;
    private int blueValue;

    public ColorSense(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.colorSensor.get("color_sensor");


       // View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        colorSensor.enableLed(false);
    }

    public void telemetryUpdate(Telemetry telemetry) {
       telemetry.addData("color", senseColor().toString());
       telemetry.addData("red", redValue);
       telemetry.addData("blue", blueValue);
        telemetry.update();
    }

    ;


    public BeaconColor senseColor() {
        redValue = colorSensor.red();
        greenValue = colorSensor.green();
        blueValue = colorSensor.blue();
        Color.RGBToHSV(redValue * 8, greenValue * 8, blueValue * 8, hsvValues); /// reading color
        if (colorSensor.red() > 1) {
            return BeaconColor.red;
        } else if (colorSensor.blue() > 1) {
            return BeaconColor.blue;
        } else {
            return BeaconColor.neither;
        }

    }
}








