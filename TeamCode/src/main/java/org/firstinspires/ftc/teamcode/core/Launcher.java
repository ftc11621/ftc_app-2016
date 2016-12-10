package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by HeavenDog on 11/12/2016.
 */

public class Launcher {
    private double power = 0.75;
    private DcMotor launcherMotor = null;
    private ElapsedTime runtime = new ElapsedTime();
    private Integer initialLauncherPosition = null;

    private Integer oneTurn = 240; //240 is 1440/6
    public Launcher(HardwareMap hardwareMap){
        this.launcherMotor = hardwareMap.dcMotor.get("motor_launcher");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Set current position to 0
        launcherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        initialLauncherPosition = launcherMotor.getCurrentPosition();
    }

    public void shoot() {
        runtime.reset();
        launcherMotor.setTargetPosition(initialLauncherPosition + (int)(1 * oneTurn)); // 1.5 revolution to shoot
        launcherMotor.setPower(power);
        while (runtime.seconds() < 2.0 && launcherMotor.isBusy()) {
            // while still spinning
        }

        launcherMotor.setPower(0.0);

        runtime.reset();
        // resume to the initial launcher position, ready to launch again
        launcherMotor.setTargetPosition(initialLauncherPosition);
        launcherMotor.setPower(0.3);
        while (runtime.seconds() < 3.0 && launcherMotor.isBusy()) {
            // while still spinning
        }
        runtime.reset();    // below to self-adjust after overshooting
        while(runtime.seconds()<2.0) {
            // self-adjusting until it is within 10 encorder counts of initial position or time out
        }

        resetLauncher();

    }

    public void setPower(double power){
        this.power = power;
    }
    public void resetLauncher(){
        launcherMotor.setPower(0.0);
        //launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //launcherMotor.setDirection(DcMotor.Direction.REVERSE);  // ready to shoot next time
    }


}
