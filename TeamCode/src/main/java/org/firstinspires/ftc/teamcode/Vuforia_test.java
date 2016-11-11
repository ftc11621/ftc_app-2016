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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


@Autonomous(name="Vuforia 11621 Test", group ="Examples")
//@Disabled
public class Vuforia_test extends LinearOpMode {

    private static final boolean alliance = true;   // true=Blue alliance, false=Red alliance
    //private RobotDriver robotDriver = null;
    private VuforiaNav vuforia_navigate = null;
    private  VuforiaTrackable wheels = null;
    private VuforiaTrackable legos = null;
    //private OpenGLMatrix lastRobotLocation = null;
    private ElapsedTime runtime = new ElapsedTime();

   @Override public void runOpMode() {
       vuforia_navigate = new VuforiaNav(alliance);     // true=Blue alliance, false=Red alliance

       waitForStart();

       vuforia_navigate.targets.activate();  // start vuforia tracking
       //robotDriver = new RobotDriver(this,hardwareMap);


      while (opModeIsActive()) {

          if(alliance) {        // Blue alliance
              if(vuforia_navigate.isWheel_visible() || vuforia_navigate.isLego_visible()) {      // if wheels is visible
                  if(vuforia_navigate.updateRobotLocation()) {
                      runtime.reset();
                  }
                  //if (vuforia_navigate.updateRobotLocation()) {     // if the current robot location available
                  telemetry.addData("Time position last updated (msec)" , "%.0f" , runtime.milliseconds());
                  telemetry.addData("X = ", "%.0f", vuforia_navigate.getX());       // x coordinate
                  telemetry.addData("Y = ", "%.0f", vuforia_navigate.getY());       // y coordinate
                  telemetry.addData("Angle from X-axis = " , "%.1f", vuforia_navigate.getOrientation(3));

                  // specify the destination coordinates, in this case X,Y of the Wheels
                  telemetry.addData("Distance to Wheels = ", "%.0f",  vuforia_navigate.getDestinationDistance(12*25.4, (12*12 - 2) * 25.4/2.0));
                  // angle > 0 when the robot has to turn right, set the coordinate of the destination
                  telemetry.addData("Angle robot needs to turn toward Wheels= ", "%.1f",  vuforia_navigate.get_robot_need_to_turn_Angle(12*25.4, (12*12 - 2) * 25.4/2.0));

                  telemetry.addData("Distance to Legos = ", "%.0f", vuforia_navigate.getDestinationDistance(-36*25.4, (12*12 - 2) * 25.4/2.0));
                  telemetry.addData("Angle robot needs to turn toward Legos= ", "%.1f",  vuforia_navigate.get_robot_need_to_turn_Angle(-36*25.4, (12*12 - 2) * 25.4/2.0));

                  //telemetry.addData("1st Angle = ", vuforia_navigate.getOrientation(1));       // 1st angle depends how the phone is oriented
                  //telemetry.addData("2nd Angle = ", vuforia_navigate.getOrientation(2));       // 2nd angle depends how the phone is oriented
                  // telemetry.addData("3rd Angle = ", vuforia_navigate.getOrientation(3));       // 3rd angle depends how the phone is oriented

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
