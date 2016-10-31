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
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 Autonomous toward a wall, when it's close turn 90 degree left
 */

@Autonomous(name="Auto Find Beacon 1", group="Examples")  // @Autonomous(...) is the other common choice
//@Disabled
public class Auto_find_beacon extends LinearOpMode {

    static final double     SPEED1                  = 0.05;
    static final double     SPEED2                  = 0.07;
    static final double     SPEED3                  = 0.1;
    static final double     SPEED4                  = 0.15;
    static final double     SPEED_ADJUSTMENT        = 1.6;     // compensating week left motor
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 9.15 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.2;
    static final double     WHEELS_SPACING_CM       = 34.6;     // spacing between wheels for turns
    static final boolean    WALL_SIDE               = true;     // true=right wall, false=left wall

    // Range ultrasonic sensors
    private RangeSensor rangeSensors = null;

    //byte[] range_Cache; //The read will return an array of bytes. They are stored in this variable
   // public static final int RANGE_REG_START = 0x04; //Register to start reading
    //public static final int RANGE_READ_LENGTH = 2; //Number of byte to read
    //public I2cDevice RANGE_front_right, RANGE_rear_right, RANGE_front;
    //public I2cDeviceSynchImpl RANGE_front_right_Reader, RANGE_rear_right_Reader, RANGE_front_Reader;


    OpticalDistanceSensor odsSensor;
    ColorSensor colorSensor;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot Configuration names ========================================
        leftMotor  = hardwareMap.dcMotor.get("motor_2");
        rightMotor = hardwareMap.dcMotor.get("motor_1");

        rangeSensors = new RangeSensor(hardwareMap);

        odsSensor = hardwareMap.opticalDistanceSensor.get("opticalSensor_1");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        // Color sensor set up
        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;   // values is a reference to the hsvValues array.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
        colorSensor.enableLed(false);

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Calibrating the light sensor
        double optical_floor = odsSensor.getRawLightDetected();
        telemetry.addData("Light sensor Raw",  optical_floor);
        telemetry.addData("Normal", odsSensor.getLightDetected());


        // Initial Range Sensor readings
        telemetry.addData("Front distance",  rangeSensors.get_Front_distance(255));
        telemetry.addData("Right Front distance",  rangeSensors.get_Front_Right_distance(255) );
        telemetry.addData("Right Rear distance",  rangeSensors.get_Rear_Right_distance(255) );

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        ///////////////////////////////////////////////////////////
        // WRITE AUTONOMOUS sequence below ===========================================
        ///////////////////////////////////////////////////////////



        // First move forward to 25 cm distance to a wall
        runtime.reset();
        double target_distance = 30.0;   // target distance
        int range_front_CM = rangeSensors.get_Front_distance(255);
        while (range_front_CM == 255 && runtime.seconds()<2) {
            range_front_CM = rangeSensors.get_Front_distance(255);
            sleep(50);
        }
        double motor_need_to_go_distance = range_front_CM - target_distance;
        while (motor_need_to_go_distance > 0) {
            encoderDrive(DRIVE_SPEED, motor_need_to_go_distance , motor_need_to_go_distance, 30.0);  // S1: Forward 48cm with 5 Sec timeout
            range_front_CM = rangeSensors.get_Front_distance(255);
            motor_need_to_go_distance = range_front_CM - target_distance;

            telemetry.addData("Front distance: " , range_front_CM);
            telemetry.update();
        }


        // Spin 90 degree, check wheel diameter and spacing if it's not accurate
        Spin_degree(-90.0);      // negative angle to spin left

/*
        // Then move along the wall to find a white line by a beacon
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        // run until white line is found or 10 sec timeouts
        //while((odsSensor.getRawLightDetected() < 1.5*optical_floor) && (runtime.seconds() < 10.0)) {
        while(runtime.seconds()<15) {
            //telemetry.addData("Look for white line, Range: ", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
           Move_along_the_wall(WALL_SIDE, 15.0, 20.0);  // right/left, lower limit, upper limit in CM
            //sleep(200);
        }
        */

        leftMotor.setPower(0.0);    // stop motors
        rightMotor.setPower(0.0);
        range_front_CM = rangeSensors.get_Front_distance(1);
        telemetry.addData("Front Range: ", range_front_CM);
        telemetry.update();

/*
        // Looking for beacon color, red in this case
        // convert the RGB values to HSV values from the color sensor
        runtime.reset();
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        while ((colorSensor.red() < 2) && (runtime.seconds() < 30.0)) {   // looking for red beacon or timeout
            // add motor movement below until the color is detected

            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Blue ", colorSensor.blue());

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        }
        leftMotor.setPower(0); // stop the motors
        rightMotor.setPower(0);
*/

        // End of AUTONOMOUS sequence ================================================
        //////////////////////////////////////////////////////////////////////////

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
        //    telemetry.addData("Status", "Run Time: " + runtime.toString());
        //    telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    // ===========================================================
    /* Method to spin a degree along the wall
     */

    private void Spin_degree(double angle_turn) {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double wheels_turn_cm = 3.14*WHEELS_SPACING_CM * angle_turn/360.0; // wheels distance to turn to the angle
        encoderDrive(TURN_SPEED, wheels_turn_cm, -wheels_turn_cm, 10.0);  // spin with 10 sec timeout
    }


    // Method to make the chassis parallel to the wall
    //

    private void Align_to_wall(boolean Side, double distance_to_wall) {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    // ===========================================================
    /* Method to move along the wall, either left or right side of the wall
       Input the side of the wall and the spacing limits.
       Distances are in CM.
     */

    private void Move_along_the_wall(boolean Side, double lower_limit, double upper_limit) {
        int range_front_right_CM, range_rear_right_CM, range_front_CM;
        if(Side) {      // right side wall

            range_front_right_CM = rangeSensors.get_Front_Right_distance(254);
            telemetry.addData("Right Front: ", range_front_right_CM);
            range_rear_right_CM = rangeSensors.get_Rear_Right_distance(254);
            telemetry.addData("Right Rear: ", range_rear_right_CM);


            if (range_front_right_CM < 255 && range_rear_right_CM < 255 && range_front_right_CM > 0 && range_rear_right_CM > 0) {

                if (range_front_right_CM < lower_limit) { // too close to wall
                    if (range_rear_right_CM < lower_limit) {
                        leftMotor.setPower(SPEED1 * SPEED_ADJUSTMENT);
                        rightMotor.setPower(SPEED2);  // turn a litte to get away from the wall
                        telemetry.addData("Position: " , "speed 1 speed 2");
                    } else if (range_rear_right_CM > upper_limit) {
                        leftMotor.setPower(0.0);
                        rightMotor.setPower(SPEED3);  // turn faster to avoid collision to the wall
                        telemetry.addData("Position: " , "speed 0 speed 1");
                    } else {    // slightly off track, slowly move away from the wall
                        leftMotor.setPower(SPEED2 * SPEED_ADJUSTMENT);
                        rightMotor.setPower(SPEED3);
                        telemetry.addData("Position: " , "speed 2 speed 3");
                    }

                } else if (range_front_right_CM > upper_limit) { // too far from wall
                    if (range_rear_right_CM < lower_limit) {
                        leftMotor.setPower(SPEED2 * SPEED_ADJUSTMENT);
                        rightMotor.setPower(0.0);  // turn a litte to get closer the wall
                        telemetry.addData("Position: " , "speed 2 speed 0");
                    } else if (range_rear_right_CM > upper_limit) {
                        leftMotor.setPower(SPEED2 * SPEED_ADJUSTMENT);
                        rightMotor.setPower(SPEED1);  // turn faster to the wall
                        telemetry.addData("Position: " , "speed 2 speed 1");
                    } else {    // slightly off track, slowly move away from the wall
                        leftMotor.setPower(SPEED3 * SPEED_ADJUSTMENT);
                        rightMotor.setPower(SPEED2);
                        telemetry.addData("Position: " , "speed 3 speed 2");
                    }
                } else {        // front distance within the lower and upper limits
                    if (range_rear_right_CM < lower_limit) {
                        leftMotor.setPower(SPEED3 * SPEED_ADJUSTMENT);
                        rightMotor.setPower(SPEED2);  // turn a litte to get closer the wall
                        telemetry.addData("Position: " , "speed 3 speed 2");
                    } else if (range_rear_right_CM > upper_limit) {
                        leftMotor.setPower(SPEED3 * SPEED_ADJUSTMENT);
                        rightMotor.setPower(SPEED2);  // turn faster to the wall
                        telemetry.addData("Position: " , "speed 3 speed 2");
                    } else {    // slightly off track, slowly move away from the wall
                        leftMotor.setPower(SPEED2 * SPEED_ADJUSTMENT);
                        rightMotor.setPower(SPEED2);
                        telemetry.addData("Position: " , "speed 2 speed 2");
                    }

                }
            } else{
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
            }


        } else {        // implement left side wall

        }
        telemetry.update();
    }


    // =====================================================================
    /*
       *  Method to perfmorm a relative move, based on encoder counts.
       *  Encoders are not reset as the move is based on the current position.
       *  Move will stop if any of three conditions occur:
       *  1) Move gets to the desired position
       *  2) Move runs out of time
       *  3) Driver stops the opmode running.
       */
    private void encoderDrive(double speed,
                             double leftCM, double rightCM,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftCM * COUNTS_PER_CM);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightCM * COUNTS_PER_CM);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d",
                //        leftMotor.getCurrentPosition(),
                 //       rightMotor.getCurrentPosition());
                //telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // sleep(250);
        }
    }
}
