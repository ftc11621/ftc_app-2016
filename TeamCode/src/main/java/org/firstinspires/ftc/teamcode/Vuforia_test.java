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

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;



@Autonomous(name="Vuforia 11621 Test", group ="Examples")
//@Disabled
public class Vuforia_test extends LinearOpMode {

    private static final boolean alliance = true;   // true=Blue alliance, false=Red alliance

    private VuforiaNav vuforia_navigate = null;
    private  VuforiaTrackable wheels = null;
    private VuforiaTrackable legos = null;
    private OpenGLMatrix lastRobotLocation = null;

   @Override public void runOpMode() {
       vuforia_navigate = new VuforiaNav(alliance);     // true=Blue alliance, false=Red alliance

       waitForStart();

       vuforia_navigate.targets.activate();  // start vuforia tracking

      while (opModeIsActive()) {

          if(alliance) {        // Blue alliance
              telemetry.addData("Wheels: ", vuforia_navigate.isWheel_visible() ? "Visible" : "Not Visisble");
              telemetry.addData("Legos: ", vuforia_navigate.isLego_visible() ? "Visible" : "Not Visisble");

          } else {              // Red alliance

          }

          // Getting robot X,Y location in the Field
          OpenGLMatrix lastLocation = vuforia_navigate.getRobotLocation();
          if (lastLocation != null)    {
              telemetry.addData("Pos", vuforia_navigate.format(lastLocation));
              telemetry.addData("X = ", vuforia_navigate.getX());       // x coordinate
              telemetry.addData("Y = ", vuforia_navigate.getY());       // y coordinate
          } else {
              telemetry.addData("Pos", "Unknown");
          }
          telemetry.update();

         idle();
      }
   }
}
