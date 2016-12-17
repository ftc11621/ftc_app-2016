package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by HeavenDog on 11/12/2016.
 */

public class LedDriver {
    private DcMotor LedMotor = null;

    public LedDriver(HardwareMap hardwareMap){
        this.LedMotor = hardwareMap.dcMotor.get("LedMotor_1");
        LedMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void LedOn() {
        LedMotor.setPower(1.0);
    }

    public void LedOff() {
        LedMotor.setPower(0.0);
    }


}
