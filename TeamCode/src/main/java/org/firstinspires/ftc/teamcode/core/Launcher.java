package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by HeavenDog on 11/12/2016.
 */

public class Launcher {
    private DcMotor launcherMotor;

    private ElapsedTime runtime = new ElapsedTime();
    public Launcher(HardwareMap hardwareMap){
        this.launcherMotor = hardwareMap.dcMotor.get("motor_3");
    }

    public void shoot() {
        launcherMotor.setDirection(DcMotor.Direction.REVERSE);
        runtime.reset();
        int initial_launcher = launcherMotor.getCurrentPosition();
        launcherMotor.setTargetPosition(initial_launcher + (int)(1.5 * 1440)); // 1.5 revolution to shoot
        launcherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherMotor.setPower(1.0);
        while (runtime.seconds() < 1.0 && launcherMotor.isBusy()) {
            // while still spinning
        }
        launcherMotor.setPower(0.0);

        runtime.reset();
        // resume to the initial launcher position, ready to launch again
        launcherMotor.setDirection(DcMotor.Direction.FORWARD); // reverse direction
        launcherMotor.setTargetPosition(initial_launcher + 1440);
        launcherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherMotor.setPower(0.1);
        while (runtime.seconds() < 5.0 && launcherMotor.isBusy()) {
            // while still spinning
        }
        launcherMotor.setPower(0.0);
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
