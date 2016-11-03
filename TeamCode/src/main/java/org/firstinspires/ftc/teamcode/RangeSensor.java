package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import static android.os.SystemClock.sleep;


public class RangeSensor {

    private static final int RANGE_REG_START = 0x04; //Register to start reading
    private static final int RANGE_READ_LENGTH = 2; //Number of byte to read
    private I2cDevice RANGE_right, RANGE_left, RANGE_front;
    private I2cDeviceSynchImpl RANGE_right_Reader, RANGE_left_Reader, RANGE_front_Reader;


    // Constructor
    public RangeSensor(HardwareMap hardwareMap){
        RANGE_front       = hardwareMap.i2cDevice.get("rangeSensor_1");
        RANGE_right = hardwareMap.i2cDevice.get("rangeSensor_2");
        RANGE_left = hardwareMap.i2cDevice.get("rangeSensor_3");

        RANGE_front_Reader       = new I2cDeviceSynchImpl(RANGE_front      , I2cAddr.create8bit(0x28), false);
        RANGE_right_Reader = new I2cDeviceSynchImpl(RANGE_right, I2cAddr.create8bit(0x26), false);
        RANGE_left_Reader  = new I2cDeviceSynchImpl(RANGE_left, I2cAddr.create8bit(0x30), false);

        RANGE_front_Reader.engage();
        RANGE_right_Reader.engage();
        RANGE_left_Reader.engage();
    }

    public int getFrontDistance(int max_distance) {
        return getSensorDistance(max_distance, RANGE_front_Reader);
    }

    private int getSensorDistance(int max_distance, I2cDeviceSynchImpl reader) {
        byte[] range_Cache;

        range_Cache = reader.read(RANGE_REG_START, RANGE_READ_LENGTH);
        int dist = range_Cache[0] & 0xFF;
        while(dist > max_distance) {
            range_Cache = reader.read(RANGE_REG_START, RANGE_READ_LENGTH);
            dist = range_Cache[0] & 0xFF;
            sleep(20);
        }
        return dist;
    }

    public int getRightDistance(int max_distance) {
        int dist = getSensorDistance(max_distance, RANGE_right_Reader);
        return dist;
    }

    public int getLeftDistance(int max_distance) {
        int dist = getSensorDistance(max_distance, RANGE_left_Reader);
        return dist;
    }

    // median method if needed
    private static int median_int(int[] m) {
        // sort the array first
        for(int nn=0; nn<m.length-1; nn++) {
            for(int mm=1; mm<m.length; mm++) {
                if(m[mm] < m[nn]){
                    int temp = m[nn];
                    m[nn] = m[mm];
                    m[mm] = temp;
                }
            }
        }
        int middle = m.length/2;
        return m[middle];
    }

}
