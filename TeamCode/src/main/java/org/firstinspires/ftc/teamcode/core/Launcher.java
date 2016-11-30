package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by HeavenDog on 11/12/2016.
 */

public class Launcher {
    private DcMotor launcherMotor = null;
    Servo servo_launcher = null;
    private ElapsedTime runtime = new ElapsedTime();

    public Launcher(HardwareMap hardwareMap){
        this.launcherMotor = hardwareMap.dcMotor.get("motor_launcher");
        //this.servo_launcher = hardwareMap.servo.get("servo_launcher");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shoot() {
        runtime.reset();
        int initial_launcher = launcherMotor.getCurrentPosition();
        launcherMotor.setTargetPosition(initial_launcher + (int)(1.5 * 1440/6)); // 1.5 revolution to shoot
        launcherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherMotor.setPower(0.5);
        while (runtime.seconds() < 5.0 && launcherMotor.isBusy()) {
            // while still spinning
        }
        for(int nn=20 ; nn > 1; nn--) {   // for smooth stopping to protect the motor or gear box
            launcherMotor.setPower((double)nn*0.05);
        }
        launcherMotor.setPower(0.0);

        runtime.reset();
        // resume to the initial launcher position, ready to launch again
        launcherMotor.setDirection(DcMotor.Direction.FORWARD); // reverse direction
        launcherMotor.setTargetPosition(initial_launcher - (int)(1.0 * 1440/6) );
        launcherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherMotor.setPower(0.1);
        while (runtime.seconds() < 5.0 && launcherMotor.isBusy()) {
            // while still spinning
        }

        resetLauncher();

    }

    public void resetLauncher(){
        launcherMotor.setPower(0.0);
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setDirection(DcMotor.Direction.REVERSE);  // ready to shoot next time
    }

    public void loadParticle() {
        for(int nn = 0 ; nn<9 ; nn++) {
            //servo_launcher.setPosition((double)nn * 0.1);
        }
    }
    public void closeParticle_entry() {
        for(int nn = 9 ; nn> 0 ; nn--) {
            //servo_launcher.setPosition((double)nn * 0.1);
        }
    }
}
