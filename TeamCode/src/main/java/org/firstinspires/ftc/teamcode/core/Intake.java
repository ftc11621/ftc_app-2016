package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by HeavenDog on 11/12/2016.
 */

public class Intake {
    private DcMotor intakeMotor;

    private ElapsedTime runtime = new ElapsedTime();
    public Intake(HardwareMap hardwareMap){
        this.intakeMotor = hardwareMap.dcMotor.get("motor_intake");
    }

    public void takein() {
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        spin();
    }
    public void kickout() {
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        spin();


    }

    private void spin() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(1.0);
        runtime.reset();
        while (runtime.seconds() < 5.0 && intakeMotor.isBusy()) {
            // while still spinning
        }
        intakeMotor.setPower(0.0);
        runtime.reset();
        // resume to the initial launcher position, ready to launch again
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}

