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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.core.ButtonPusher;
import org.firstinspires.ftc.teamcode.core.ParticleDoor;
import org.firstinspires.ftc.teamcode.core.RobotDriver;
import org.firstinspires.ftc.teamcode.core.VuforiaSensor;


@Autonomous(name="Vuforia Navigation Blue", group ="Competition")
//@Disabled
public class Autonomous_Mode extends BaseNavigation {

     public void navigate() {

         ParticleDoor pDoor = new ParticleDoor(hardwareMap);
         baseLog("Launcher:","Shoot Launcher");
         //go to the shooting position
         moveToPosition(36 * 25.4, 36 * 25.4);
         baseLog("Robot;","Stop");
         baseLog("Launcher","Shoot Launcher");
        //stop and shoot
         launcher.shoot();
         pDoor.openDoor(); // load another partile
         launcher.shoot();
         baseLog("Robot","Move To Position");
        //go to the beacon position
         baseLog("Robot","Open Launcher Servo");
         //open the launcher servo
         baseLog("Launcher","Shoot Launcher");
         //launcher.shoot();
         //
        moveToPosition(72*25.4 , 72* 25.4);
         baseLog("Robot","Turn To Beacon");
        //turn to face the beacon

         baseLog("Robot","Move To Beacon");
        //go near the beacon

         baseLog("Sensor","Sense Color");
        //sense the color
         baseLog("Button Pusher", "Push Correct Button");
        //push the correct button
         buttonPusher.pushButton(ButtonPusher.Button.left);





    }

    private double cameraAxleDistance = 200;
    private double axisCameraDistance = 25;
    public void moveToPosition(double destination_x, double destination_y, VuforiaSensor vuforia) {
        double distance = vuforia.getDestinationDistance(destination_x, destination_y);
        double toAngle = vuforia.getRobotNeedToTurnAngle(destination_x, destination_y);
        double fromAngle = vuforia.getOrientation(3);
        robotDriver.go(RobotDriver.Speed.speed5,cameraAxleDistance);
        robotDriver.turnToAngle(fromAngle,toAngle);
        robotDriver.go(RobotDriver.Speed.normal, distance - cameraAxleDistance);
        stop();

    }
}
