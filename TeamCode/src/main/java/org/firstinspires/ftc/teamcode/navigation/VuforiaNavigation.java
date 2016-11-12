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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.ButtonPusher;
import org.firstinspires.ftc.teamcode.core.Launcher;
import org.firstinspires.ftc.teamcode.core.RobotDriver;
import org.firstinspires.ftc.teamcode.core.VuforiaSensor;


@Autonomous(name="Vuforia Navigation Binh", group ="Competition")
//@Disabled
public class VuforiaNavigation extends LinearOpMode {





    RobotDriver robotDriver;
    VuforiaSensor vuforia;
    Launcher launcher;
    @Override public void runOpMode() {

        vuforia = new VuforiaSensor(true);
        robotDriver = new RobotDriver(this,hardwareMap);
        launcher = new Launcher(hardwareMap);
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        vuforia.activate();
        //go to the shooting position
        double distance = vuforia.getDestinationDistance(36*25.4,36*25.4);
        double toAngle = vuforia.getRobotNeedToTurnAngle(36*25.4,36*25.4);
        double fromAngle =vuforia.getOrientation(3);
        robotDriver.turnToAngle(fromAngle,toAngle);

        robotDriver.go(RobotDriver.Speed.normal, distance);
        //stop and shoot
        launcher.shoot();
        //go to the beacon position
        //stop
        //turn to face the beacon
        //go near the beacon
        //sense the color
        //push the correct button
        ButtonPusher buttonPusher = new ButtonPusher();
        buttonPusher.pushButton(ButtonPusher.Button.left);





    }



}