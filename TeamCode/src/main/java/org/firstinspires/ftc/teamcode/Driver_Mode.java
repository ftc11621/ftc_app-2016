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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 This is the final code for Driver Mode in the competition
 */

@TeleOp(name="Driver Mode", group="Competition")  // @Autonomous(...) is the other common choice
//@Disabled
public class Driver_Mode extends OpMode
{
    /* Declare OpMode members. */
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 9.15 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.1;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor launcherMotor = null;
    private DcMotor intakeMotor = null;
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    double MaxDcPower = 0.07;       // chassis motors speeds

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        //launcherMotor  = hardwareMap.dcMotor.get("motor_launcher");
        //intakeMotor =hardwareMap.dcMotor.get("motor_intake");
        leftMotor  = hardwareMap.dcMotor.get("motor_2");
        rightMotor = hardwareMap.dcMotor.get("motor_1");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //launcherMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
        //launcherMotor.setPower(0.0);
        //intakeMotor.setPower(0.0);
        //launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //telemetry.addData("Status", "Running: " + runtime.toString());


        if (gamepad2.y) {                   // shoot launcher
            //launcherMotor.setPower(1.0);
        } else if(gamepad2.a) {             // spin Intake
            //intakeMotor.setPower(1.0);
        } else if(gamepad2.b) {             // semi-autonomous beacon claiming
            // add code that works with autonomous beacon claiming here.
        } else if(gamepad1.dpad_down) {        // Chassis maximum motors power
            MaxDcPower = 0.07;
        } else if(gamepad1.dpad_left) {
            MaxDcPower = 0.1;
        } else if(gamepad1.dpad_up) {
            MaxDcPower = 0.20;
        } else if(gamepad1.dpad_right) {        // Last chassis maximum power setting
            MaxDcPower = 1.0;
        } else {
            //launcherMotor.setPower(0);
            //intakeMotor.setPower(0);
        }
        // add beacon claiming servo motor






        leftMotor.setPower(-gamepad1.left_stick_y * MaxDcPower);    // tank style joysticks
        rightMotor.setPower(-gamepad1.right_stick_y * MaxDcPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
