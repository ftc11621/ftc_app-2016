package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Marie on 10/25/2016.
 */

public class RangeSensor {
    //byte[] range_Cache; //The read will return an array of bytes. They are stored in this variable
    private static final int RANGE_REG_START = 0x04; //Register to start reading
    private static final int RANGE_READ_LENGTH = 2; //Number of byte to read
    private I2cDevice RANGE_front_right, RANGE_rear_right, RANGE_front;
    private I2cDeviceSynchImpl RANGE_front_right_Reader, RANGE_rear_right_Reader, RANGE_front_Reader;
    //int range_front_right_CM, range_rear_right_CM, range_front_CM;

    // Constructor
    public RangeSensor(HardwareMap hardwareMap){
        RANGE_front       = hardwareMap.i2cDevice.get("rangeSensor_1");
        RANGE_front_right = hardwareMap.i2cDevice.get("rangeSensor_2");
        RANGE_rear_right  = hardwareMap.i2cDevice.get("rangeSensor_3");

        RANGE_front_Reader       = new I2cDeviceSynchImpl(RANGE_front      , I2cAddr.create8bit(0x28), false);
        RANGE_front_right_Reader = new I2cDeviceSynchImpl(RANGE_front_right, I2cAddr.create8bit(0x26), false);
        RANGE_rear_right_Reader  = new I2cDeviceSynchImpl(RANGE_rear_right , I2cAddr.create8bit(0x30), false);

        RANGE_front_Reader.engage();
        RANGE_front_right_Reader.engage();
        RANGE_rear_right_Reader.engage();
    }

    public int get_Front_distance() {
        byte[] range_Cache;
        range_Cache = RANGE_front_Reader.read(RANGE_REG_START, RANGE_READ_LENGTH);
        return range_Cache[0] & 0xFF;
    }

    public int get_Front_Right_distance() {
        byte[] range_Cache;
        range_Cache = RANGE_front_right_Reader.read(RANGE_REG_START, RANGE_READ_LENGTH);
        return range_Cache[0] & 0xFF;
    }

    public int get_Rear_Right_distance() {
        byte[] range_Cache;
        range_Cache = RANGE_rear_right_Reader.read(RANGE_REG_START, RANGE_READ_LENGTH);
        return range_Cache[0] & 0xFF;
    }
}
