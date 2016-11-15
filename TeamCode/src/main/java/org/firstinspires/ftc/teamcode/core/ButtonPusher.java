package org.firstinspires.ftc.teamcode.core;

/**
 * Created by Gary on 11/12/16.
 */

public class ButtonPusher {

    public enum Button {left,right}

    public void pushButton(Button button){
        if(Button.left.equals(button)){
            //Do the logic to move the servo to the left
        }
        else {
            //Do the logic to move the servo to the right
        }
    }
}
