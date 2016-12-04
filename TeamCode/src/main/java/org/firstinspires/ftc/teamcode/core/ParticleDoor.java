package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Marie on 12/3/2016.
 */

public class ParticleDoor {
    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 0.6;     // Maximum rotational position
    static final double MIN_POS = 0.2;     // Minimum rotational position
    // Define class members
    Servo servo;
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position


    public enum Button {left, right}

    public ParticleDoor(HardwareMap hardwareMap) {
        servo = hardwareMap.servo.get("servo_launcher");
        closeDoor();
    }

    public boolean isOpen() {

        return position == MAX_POS;
    }

    public void openDoor() {
        position = MAX_POS;
        servo.setPosition(MAX_POS);
    }

    public void closeDoor() {
        position = MIN_POS;
        servo.setPosition(MIN_POS);
    }


}
