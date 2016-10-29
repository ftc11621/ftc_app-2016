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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 Autonomous toward a wall, when it's close turn 90 degree left
 */

@Autonomous(name="Auto Find white line", group="Examples")  // @Autonomous(...) is the other common choice
@Disabled
public class Auto_ultrasonic_ex3 extends LinearOpMode {

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 9.15 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.2;
    static final double     WHEELS_SPACING_CM       = 34.3;     // spacing between wheels

    ModernRoboticsI2cRangeSensor rangeSensor;
    OpticalDistanceSensor odsSensor;  // Hardware Device Object

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor  = hardwareMap.dcMotor.get("motor_2");
        rightMotor = hardwareMap.dcMotor.get("motor_1");

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor_1");
        odsSensor = hardwareMap.opticalDistanceSensor.get("opticalSensor_1");


        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.addData("Range: ", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        // Calibrating the light sensor
        double optical_floor = odsSensor.getRawLightDetected();
        telemetry.addData("Raw",  optical_floor);
        telemetry.addData("Normal", odsSensor.getLightDetected());


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ///////////////////////////////////////////////////////////
        // WRITE AUTONOMOUS sequence below ===========================================

        // First move forward to 30 cm distance to a wall
        double target_distance = 25.0;   // 10 cm target distance
        double motor_need_to_go_distance = rangeSensor.getDistance(DistanceUnit.CM) - target_distance;

        while (motor_need_to_go_distance > 0) {
            encoderDrive(DRIVE_SPEED, motor_need_to_go_distance , motor_need_to_go_distance, 30.0);  // S1: Forward 48cm with 5 Sec timeout
            motor_need_to_go_distance = rangeSensor.getDistance(DistanceUnit.CM) - target_distance;

            telemetry.addData("Range: ", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }


        // Turn left 90 degree, check wheel diameter and spacing if it's not accurate
        double angle_turn = -90.0;     // positive to turn right
        double wheels_turn_cm = 3.14*WHEELS_SPACING_CM * angle_turn/360.0; // wheels distance to turn to the angle
        encoderDrive(TURN_SPEED, wheels_turn_cm, -wheels_turn_cm, 30.0);
        // Then move forward to find a white line by a beacon
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(0.1);
        rightMotor.setPower(0.1);
        runtime.reset();
        // run until white line is found or 10 sec timeouts
        while((odsSensor.getRawLightDetected() < 1.5*optical_floor) && (runtime.seconds() < 10.0)) {
            telemetry.addData("Look for white line, Range: ", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            sleep(200);
        }
        telemetry.addData("White line found, Range: ", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        leftMotor.setPower(0); // stop the motors
        rightMotor.setPower(0);

        // End of AUTONOMOUS sequence ================================================
        //////////////////////////////////////////////////////////////////////////

        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive()) {
        //    telemetry.addData("Status", "Run Time: " + runtime.toString());
        //    telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);

        //    idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
       // }
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
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
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
