/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 Autonomous toward a wall, when it's close turn 90 degree left
 */

@Autonomous(name="Auto Find Beacon", group="Examples")  // @Autonomous(...) is the other common choice
//@Disabled
public class Auto_find_beaconRobotDriver extends LinearOpMode {

    private RangeSensor rangeSensors = null;
    OpticalDistanceSensor odsSensor;
    ColorSensor colorSensor;

    private enum Side {right,left}

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private RobotDriver robotDriver = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot Configuration names ========================================

        robotDriver = new RobotDriver(this, hardwareMap);
        rangeSensors = new RangeSensor(hardwareMap);
        //odsSensor = hardwareMap.opticalDistanceSensor.get("opticalSensor_1");
        //colorSensor = hardwareMap.colorSensor.get("sensor_color");

        // Color sensor set up
        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;   // values is a reference to the hsvValues array.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
        //colorSensor.enableLed(false);


        idle();
        robotDriver.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Calibrating the light sensor
        /*
        double optical_floor = odsSensor.getRawLightDetected();
        telemetry.addData("Light sensor Raw",  optical_floor);
        telemetry.addData("Normal", odsSensor.getLightDetected());
        telemetry.update();
        */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ///////////////////////////////////////////////////////////
        // WRITE AUTONOMOUS sequence below ===========================================

        // First move forward to 25 cm distance to a wall
        double target_distance = 25.0;   // 10 cm target distance
        double motor_need_to_go_distance = rangeSensors.getFrontDistance(255) - target_distance;

        while (motor_need_to_go_distance > 0) {
            telemetry.addData("Distance remaining: ", motor_need_to_go_distance);
            telemetry.update();

            robotDriver.go(RobotDriver.Direction.forward, RobotDriver.Speed.normal, motor_need_to_go_distance );
            motor_need_to_go_distance = rangeSensors.getFrontDistance(200) - target_distance;
        }


        // Turn left 90 degree, check wheel diameter and spacing if it's not accurate
        robotDriver.turn(RobotDriver.Turn.left90);

/*


        // Then move along the wall to find a white line by a beacon
        robotDriver.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        // run until white line is found or 10 sec timeouts
        while((odsSensor.getRawLightDetected() < 1.5*optical_floor) && (runtime.seconds() < 10.0)) {
            //telemetry.addData("Look for white line, Range: ", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
           moveAlongWall(Side.right, 25.0, 30.0);  // right/left, lower limit, upper limit in CM
            //sleep(200);
        }
        robotDriver.stop();


        // Looking for beacon color, red in this case
        // convert the RGB values to HSV values from the color sensor
        runtime.reset();
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        while ((colorSensor.red() < 2) && (runtime.seconds() < 30.0)) {   // looking for red beacon or timeout
            // add motor movement below until the color is detected

            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Blue ", colorSensor.blue());
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        }
        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.

*/
        robotDriver.stop();
        while (opModeIsActive()) {   // wait to hit the stop button

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }



    // ===========================================================
    /* Method to move along the wall, either left or right side of the wall
       Input the side of the wall and the spacing limits.
       Distances are in CM.
//     */
//---/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void moveAlongWall(Side side, double lower_limit, double upper_limit) {
        double distance = rangeSensors.getLeftDistance(100);
        if(Side.right.equals(side)) {      // right side wall
            distance = rangeSensors.getRightDistance(100);
        }






    }


}
