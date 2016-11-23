package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

/**
 * Created by Gary on 11/12/16.
 */
@TeleOp(name = "Button Pusher", group = "Examples")
//@Disabled
public class ButtonPusher {
    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position
    // Define class members
    Servo servo;
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position


    public enum Button {left, right}

    public ButtonPusher(HardwareMap hardwareMap) {
        servo = hardwareMap.servo.get("beacon_servo");
    }

    public void pushButton(Button button) {


        // slew the servo, according to the rampUp (direction) variable.
        if (Button.left.equals(button)) {
            // Keep stepping up until we hit the max value.
            position += INCREMENT;
            if (position >= MAX_POS) {
                position = MAX_POS;

            }
        } else {
            // Keep stepping down until we hit the min value.
            position -= INCREMENT;
            if (position <= MIN_POS) {
                position = MIN_POS;

            }
        }


        // Display the current value


        // Set the servo to the new position and pause;
        servo.setPosition(position);
        servo.setPosition(0.5);

    }


}
