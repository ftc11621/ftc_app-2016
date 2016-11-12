package org.firstinspires.ftc.teamcode.reference;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Range Sensor Example", group="Examples")  // @Autonomous(...) is the other common choice
@Disabled
public class RangeSensor_example extends OpMode {
   // ModernRoboticsI2cRangeSensor rangeSensor1;  // default 0x28 address
    byte[] range_Cache; //The read will return an array of bytes. They are stored in this variable
    public static final int RANGE_REG_START = 0x04; //Register to start reading
    public static final int RANGE_READ_LENGTH = 2; //Number of byte to read
    public I2cDevice RANGE_front_right, RANGE_rear_right, RANGE_front;
    public I2cDeviceSynchImpl RANGE_front_right_Reader, RANGE_rear_right_Reader, RANGE_front_Reader;
    int range_front_right_CM, range_rear_right_CM, range_front_CM;


    @Override
    public void init() {
        //rangeSensor1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor_1"); // default 0x28
        RANGE_front       = hardwareMap.i2cDevice.get("rangeSensor_1");
        RANGE_front_right = hardwareMap.i2cDevice.get("rangeSensor_2");
        RANGE_rear_right  = hardwareMap.i2cDevice.get("rangeSensor_3");

        RANGE_front_Reader       = new I2cDeviceSynchImpl(RANGE_front      , I2cAddr.create8bit(0x28), false);
        RANGE_front_right_Reader = new I2cDeviceSynchImpl(RANGE_front_right, I2cAddr.create8bit(0x26), false);
        RANGE_rear_right_Reader  = new I2cDeviceSynchImpl(RANGE_rear_right , I2cAddr.create8bit(0x30), false);
        RANGE_front_Reader.engage();
        RANGE_front_right_Reader.engage();
        RANGE_rear_right_Reader.engage();

        telemetry.addData("rear name", RANGE_front.getDeviceName());
        telemetry.addData("rear name", RANGE_front_right.getDeviceName());
        telemetry.addData("rear name", RANGE_rear_right.getDeviceName());
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        range_Cache = RANGE_front_Reader.read(RANGE_REG_START, RANGE_READ_LENGTH);
        range_front_CM = range_Cache[0] & 0xFF;
        telemetry.addData("Front optical", range_Cache[1]& 0xFF);

        range_Cache = RANGE_front_right_Reader.read(RANGE_REG_START, RANGE_READ_LENGTH);
        range_front_right_CM = range_Cache[0] & 0xFF;
        telemetry.addData("Right Front optical", range_Cache[1]& 0xFF);

        range_Cache = RANGE_rear_right_Reader.read(RANGE_REG_START, RANGE_READ_LENGTH);
        range_rear_right_CM = range_Cache[0] & 0xFF;
        telemetry.addData("Right Rear optical", range_Cache[1]& 0xFF);

       // telemetry.addData("Front distance", rangeSensor1.getDistance(DistanceUnit.CM));
        telemetry.addData("Front distance",  range_front_CM );
        telemetry.addData("Right Front distance",  range_front_right_CM );
        telemetry.addData("Right Rear distance",  range_rear_right_CM );

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}