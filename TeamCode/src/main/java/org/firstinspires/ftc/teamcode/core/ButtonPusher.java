package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

/**
 * Created by Gary on 11/12/16.
 */
@TeleOp(name="Button Pusher", group ="Examples")
//@Disabled
public class ButtonPusher {
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    com.qualcomm.robotcore.hardware.Servo servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    public enum Button {left,right}

    public void pushButton(Button button){
        

        if(Button.left.equals(button)){
            while(position < MAX_POS){
                position += INCREMENT ;
                servo.setPosition(position);

            }
        }

                // Keep stepping up until we hit the max value.
        else {
            while (position > MIN_POS);
                position -= INCREMENT;
                servo.setPosition(position);
        }
        }
    }



