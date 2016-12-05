package org.firstinspires.ftc.teamcode.reference.camera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;


import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;




@Autonomous(name="Concept: ColorSense Sensor", group = "Concept")
@Disabled
public class ColorSensorDriver extends LinearOpMode {


    public enum ColorSensorDevice {MODERN_ROBOTICS_I2C};


    public ColorSensorDevice device = ColorSensorDevice.MODERN_ROBOTICS_I2C;


    ColorSensor colorSensor;
    DeviceInterfaceModule dim;
    LED led;
    TouchSensor t;


    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap.logDevices();


        dim = hardwareMap.deviceInterfaceModule.get("dim");
        switch (device) {
            case MODERN_ROBOTICS_I2C:
                colorSensor = hardwareMap.colorSensor.get("colorSensor");
                break;
        }
        led = hardwareMap.led.get("led");
        t = hardwareMap.touchSensor.get("t");


        waitForStart();


        float hsvValues[] = {0,0,0};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
        while (opModeIsActive()) {


            enableLed(t.isPressed());


            switch (device) {
                case MODERN_ROBOTICS_I2C:
                    Color.RGBToHSV(colorSensor.red()*8, colorSensor.green()*8, colorSensor.blue()*8, hsvValues);
                    break;
            }
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);


            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            waitOneFullHardwareCycle();
        }
    }


    private void enableLed(boolean value) {
        switch (device) {
            case MODERN_ROBOTICS_I2C:
                colorSensor.enableLed(value);
                break;
        }
    }
}

