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
package org.firstinspires.ftc.teamcode.camera;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
public class Autonomous1 extends LinearOpMode {


    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    DcMotorController controllerFR;
    DcMotorController controllerI;


    int motorBR = 1;
    int motorBL = 2;
    int motorIntake = 1;
    int motorGConveyor = 2;
    VuforiaSensor vuforia = new VuforiaSensor(0,1,0);
    @Override
    public void runOpMode() {


       /*
        * Initialize the drive system variables.
        * The init() method of the hardware class does all the work here
        */
        controllerFR = hardwareMap.dcMotorController.get("controller1");
        controllerI = hardwareMap.dcMotorController.get("controller2");


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        controllerFR.setMotorMode(motorBL,DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        controllerFR.setMotorMode(motorBR,DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Status","hello");
        telemetry.update();


        idle();


        controllerFR.setMotorMode(motorBL,DcMotor.RunMode.RUN_USING_ENCODER);
        controllerFR.setMotorMode(motorBR,DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
       /*telemetry.addData("Path0",  "Starting at %7d :%7d",
                         robot.leftMotor.getCurrentPosition(),
                         robot.rightMotor.getCurrentPosition());
       telemetry.update();
       */
        // Wait for the game to start (driver presses PLAY)


        vuforia.setupVuforia();


        waitForStart();


        vuforia.visionActiavte();


        // Step through each leg of the path,
        // + number -number forward opposite for backwards
        //+number +number right turn opposite for backwards
        // 12 for right 90 degree turn
        // 12 for left 90 degree turn
        encoderDrive(DRIVE_SPEED,  -51, 51, 7.0);  // S1: Forward 51 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   -8, -8, 4.0);  // S2: Turn Right 8 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, 48, -48, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        vuforiaDrive(7, 3);
        timeDrive(DRIVE_SPEED, .3);






        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newLeftTarget = controllerFR.getMotorCurrentPosition(2)+ (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = controllerFR.getMotorCurrentPosition(1) + (int)(rightInches * COUNTS_PER_INCH);
            controllerFR.setMotorTargetPosition(2,newLeftTarget);
            controllerFR.setMotorTargetPosition(1,newRightTarget);


            // Turn On RUN_TO_POSITION
            controllerFR.setMotorMode(2, DcMotor.RunMode.RUN_TO_POSITION);
            controllerFR.setMotorMode(1, DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();


            controllerFR.setMotorPower(2, Math.abs(speed));
            controllerFR.setMotorPower(1, Math.abs(speed));




            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (controllerFR.isBusy(2) && controllerFR.isBusy(1))) {


               /*// Display it for the driver.
               telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
               telemetry.addData("Path2",  "Running at %7d :%7d",
                                           robot.leftMotor.getCurrentPosition(),
                                           robot.rightMotor.getCurrentPosition());
               telemetry.update();
               */
            }


            // Stop all motion;
            controllerFR.setMotorPower(2,0);
            controllerFR.setMotorPower(1,0);


            // Turn off RUN_TO_POSITION
            controllerFR.setMotorMode(2, DcMotor.RunMode.RUN_USING_ENCODER);
            controllerFR.setMotorMode(1, DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void timeDrive(double speed, double time)
    {
        ElapsedTime timer = new ElapsedTime();


        timer.reset();
        controllerFR.setMotorPower(2,-speed);
        controllerFR.setMotorPower(1, speed);
        while(timer.seconds() < time ){}


        controllerFR.setMotorPower(2,0);
        controllerFR.setMotorPower(1,0);
    }
    public void vuforiaDrive(int xDis, int yDis)
    {
        while(opModeIsActive()) {
            ElapsedTime timer = new ElapsedTime();


            timer.reset();


            double angle = vuforia.getRobot("Tools Target", "angle");
            double yDist = vuforia.getRobot("Tools Target", "yAway");
            double xDist = vuforia.getRobot("Tools Target", "xAway");
            if(vuforia.isVis("Tools Target")) {
                if(yDist > 10) {
                    encoderDrive(DRIVE_SPEED, 4,-4,3.0);
                    encoderDrive(TURN_SPEED, 4, 4, 3.0);
                    encoderDrive(DRIVE_SPEED, -4,4,3.0);
                    encoderDrive(TURN_SPEED, -4, -4, 3.0);


                   /*encoderDrive(TURN_SPEED, 12, 12, 4.0);
                   encoderDrive(DRIVE_SPEED, -yDist/25.4, yDist/25.4, 4.0);
                   encoderDrive(DRIVE_SPEED, -12, -12, 4.0);
                   */


                } else if(yDist < -10){
                    encoderDrive(DRIVE_SPEED, -4,4,3.0);
                    encoderDrive(TURN_SPEED, -4, -4, 3.0);
                    encoderDrive(DRIVE_SPEED, 4,-4,3.0);
                    encoderDrive(TURN_SPEED, 4, 4, 3.0);


                   /*encoderDrive(TURN_SPEED, -12, -12, 4.0);
                   encoderDrive(DRIVE_SPEED, -yDist/25.4, yDist/25.4, 4.0);
                   encoderDrive(DRIVE_SPEED, 12, 12, 4.0);
                   */


                }
                if ((angle > 3 && angle < 90) || (angle < -90 && angle > -177)) {
                    controllerFR.setMotorPower(1, .6);
                    while(timer.seconds() < .1) {}
                    timer.reset();
                    controllerFR.setMotorPower(1, 0);
                } else if ((angle < -3 && angle > -90) || (angle > 90 && angle < 177)) {
                    controllerFR.setMotorPower(2, -.6);
                    while (timer.seconds() < .1) {}
                    timer.reset();
                    controllerFR.setMotorPower(2, 0);
                }
                if(yDist < 5 && yDist > -5 && ((angle < 3 && angle > -3) || angle > 177 || angle < -177)){
                    while(xDist > 125)
                    {
                        encoderDrive(DRIVE_SPEED, 1, -1, 2.0);
                    }
                }
               /*else
               {


               }*/
            }


        }
    }
}
