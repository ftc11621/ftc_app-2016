package org.firstinspires.ftc.teamcode.core;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by HeavenDog on 12/1/2016.
 */

public class ColorSense {
    ColorSensor colorSensor;
    float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues; // values is a reference to the hsvValues array.
    View relativeLayout;

    public ColorSense(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        //float hsvValues[] = {0F,0F,0F};

        View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        colorSensor.enableLed(false);
    }

    public static enum Beacon_colors {red, blue, neither};


    public Beacon_colors senseColor() {
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues); /// reading color
        if (colorSensor.red() > 1) {
            return Beacon_colors.red;
        } else if (colorSensor.blue() > 1) {
            return Beacon_colors.blue;
        } else {
            return Beacon_colors.neither;
        }
        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        //relativeLayout.post(new Runnable() {
        //    public void run() {
        //        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        //    }
        //});
    }
}








