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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.core.VuforiaSensor;


@Autonomous(name="Vuforia 11621 Test", group ="Examples")
@Disabled
public class Vuforia_test extends LinearOpMode {

    private static final boolean alliance = true;   // true=Blue alliance, false=Red alliance
    //private RobotDriver robotDriver = null;
    private VuforiaSensor vuforiaSensor = null;
    private  VuforiaTrackable wheels = null;
    private VuforiaTrackable legos = null;
    //private OpenGLMatrix lastRobotLocation = null;
    private ElapsedTime runtime = new ElapsedTime();

   @Override public void runOpMode() {
       vuforiaSensor = new VuforiaSensor();     // true=Blue alliance, false=Red alliance

       waitForStart();

       vuforiaSensor.activate();  // start vuforia tracking
       //robotDriver = new RobotDriver(this,hardwareMap);


      while (opModeIsActive()) {

          if(alliance) {        // Blue alliance
              if(vuforiaSensor.isWheel_visible() || vuforiaSensor.isLego_visible()) {      // if wheels is visible
                  if(vuforiaSensor.updateRobotLocation()) {
                      runtime.reset();
                  }
                  //if (vuforiaSensor.updateRobotLocation()) {     // if the current robot location available
                  telemetry.addData("Time position last updated (msec)" , "%.0f" , runtime.milliseconds());
                  telemetry.addData("X = ", "%.0f", vuforiaSensor.getX());       // x coordinate
                  telemetry.addData("Y = ", "%.0f", vuforiaSensor.getY());       // y coordinate
                  telemetry.addData("Angle from X-axis = " , "%.1f", vuforiaSensor.getOrientation(3));

                  // specify the destination coordinates, in this case X,Y of the Wheels
                  telemetry.addData("Distance to Wheels = ", "%.0f",  vuforiaSensor.getDestinationDistance(12*25.4, (12*12 - 2) * 25.4/2.0));
                  // angle > 0 when the robot has to turn right, set the coordinate of the destination
                  telemetry.addData("Angle robot needs to turn toward Wheels= ", "%.1f",  vuforiaSensor.getRobotNeedToTurnAngle(12*25.4, (12*12 - 2) * 25.4/2.0));

                  telemetry.addData("Distance to Legos = ", "%.0f", vuforiaSensor.getDestinationDistance(-36*25.4, (12*12 - 2) * 25.4/2.0));
                  telemetry.addData("Angle robot needs to turn toward Legos= ", "%.1f",  vuforiaSensor.getRobotNeedToTurnAngle(-36*25.4, (12*12 - 2) * 25.4/2.0));

                  //telemetry.addData("1st Angle = ", vuforiaSensor.getOrientation(1));       // 1st angle depends how the phone is oriented
                  //telemetry.addData("2nd Angle = ", vuforiaSensor.getOrientation(2));       // 2nd angle depends how the phone
                  //
                  // is oriented
                  // telemetry.addData("3rd Angle = ", vuforiaSensor.getOrientation(3));       // 3rd angle depends how the phone is oriented

              } else {
                  telemetry.addData("Wheels or Legos: ", "Not Visible");
              }

          } else {              // Red alliance

          }

          telemetry.update();

          //robotDriver.goStraight(RobotDriver.Speed.normal);

         idle();
      }
   }
}
