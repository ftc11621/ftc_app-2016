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
package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.ButtonPusher;
import org.firstinspires.ftc.teamcode.core.Intake;
import org.firstinspires.ftc.teamcode.core.Launcher;
import org.firstinspires.ftc.teamcode.core.ParticleDoor;
import org.firstinspires.ftc.teamcode.core.RobotDriver;

/**
 This is the final code for Driver Mode in the competition
 */

@TeleOp(name="Driver Mode", group="Competition")  // @Autonomous(...) is the other common choice
//@Disabled
public class Driver_Mode extends OpMode
{
    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();

    RobotDriver robotDriver;
    Launcher launcher;
    Intake intake;
    ButtonPusher buttonPusher;
    ParticleDoor pDoor;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        robotDriver = new RobotDriver(hardwareMap);
        launcher = new Launcher(hardwareMap);
        intake = new Intake (hardwareMap);
        buttonPusher = new ButtonPusher(hardwareMap);
        pDoor = new ParticleDoor(hardwareMap);

    }

    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robotDriver.setSpeed(RobotDriver.Speed.speed1);
        runtime.reset();
        launcher.resetLauncher();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //telemetry.addData("Driver Mode", "In loop");
        //telemetry.update();

        gamepadDriver();
        gamepadGunner();
}

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    private void gamepadDriver(){
        if(gamepad1.dpad_down) {        // Chassis maximum motors power
            robotDriver.setSpeed(RobotDriver.Speed.speed1);
        } else if(gamepad1.dpad_left) {
            robotDriver.setSpeed(RobotDriver.Speed.speed3);
        } else if(gamepad1.dpad_up) {
            robotDriver.setSpeed(RobotDriver.Speed.speed5);
        } else if(gamepad1.dpad_right) {        // Last chassis maximum power setting
            robotDriver.setSpeed(RobotDriver.Speed.speed10);
        } else if(gamepad1.left_trigger > 0) {
            robotDriver.backwards();
        }
        else if (gamepad1.right_trigger > 0){
            robotDriver.forward();
        }
        robotDriver.turn(-gamepad1.left_stick_y* robotDriver.getSpeed().getSpeed(), -gamepad1.right_stick_y*robotDriver.getSpeed().getSpeed()); //tank style joysticks

    }
    private void gamepadGunner(){
        if(gamepad2.x){
            pDoor.closeDoor();
        }
        else if (gamepad2.b){
            pDoor.openDoor();
        }
        if(gamepad2.right_stick_y > 0) {// spin Intake when pressed and hold "A" button
            intake.setPower(gamepad2.right_stick_y);
            intake.takein();
        } else if (gamepad2.right_stick_y < 0){                      // stop Intake
            intake.setPower(gamepad2.right_stick_y * -1);
            intake.kickout();

        }
        else {
            intake.stop();
        }


        if (gamepad2.y) {                   // run launcher
            launcher.shoot();
        } //else if(gamepad2.b) {             // semi-autonomous beacon claiming
            //buttonPusher.pushButton(ButtonPusher.Button.left);
            // add code that works with autonomous beacon claiming here.
        //}
    }
}
