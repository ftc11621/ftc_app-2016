package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.ColorSense;
import org.firstinspires.ftc.teamcode.core.ParticleDoor;
import org.firstinspires.ftc.teamcode.core.RobotDriver;

/**
 * Created by HeavenDog on 12/4/2016.
 */

@Autonomous(name="Autonomous Red Blind 1", group ="Competition")
//@Disabled
public class RedAndBlind extends BaseNavigation {


    public void navigate() {
        ParticleDoor pDoor = new ParticleDoor(hardwareMap); // on top to make sure it opens before interrup

        ElapsedTime runtime_navigate = new ElapsedTime();

        moveAndShoot(-30.0);     // negative for intake front

        /*
        robotDriver.go(RobotDriver.Speed.speed3, -30 * 2.54); // negative for intake front

        launcher.shoot();   // shoot 1st particle
        pDoor.openDoor();
        runtime_navigate.reset();
        while (runtime_navigate.seconds() < 1.0) {    // wait for the chance the door is fully open
        }
        launcher.shoot();   // shoot 2nd particle
       */




        // Running into the Cap ball
        /*
        robotDriver.turnToAngle(0, -90);  // turn toward the cap ball
        robotDriver.go(RobotDriver.Speed.speed3, 36.0*2.54); // run straight to cap ball
        robotDriver.go(RobotDriver.Speed.speed3, -12.0*2.54); // backup a little
        robotDriver.turnToAngle(0, -90);  // turn approximately toward the gears
        */

        /*
        // blind encoder move to get closer to a beacon
        robotDriver.turnToAngle(0, -150);
        robotDriver.go(RobotDriver.Speed.speed3, 40*2.54);
        robotDriver.turnToAngle(0,-60);
        */

        // Going to beacon
        if (moveToPosition(Vuforia_gears_x + 200, Vuforia_gears_y)) { // 300 mm from gears
            telemetry.addData("X" , vuforia.getX());
            telemetry.addData("Y", vuforia.getY());
            telemetry.addData("Distance to Gears", "%.0f", vuforia.getDestinationDistance(Vuforia_gears_x,Vuforia_gears_y));
            telemetry.addData("Angle to Gears", "%.0f", vuforia.getRobotNeedToTurnAngle(Vuforia_gears_x,Vuforia_gears_y));
            telemetry.update();
            // Now all the way to the beacon
            if (moveToPosition(Vuforia_gears_x + 50, Vuforia_gears_y)) { // 50mm
                sleep(1000);  // just in case the color sensor needs time

                // read beacon's color

                // below for Red alliance
                /*
                ColorSense beaconColor = new ColorSense(hardwareMap);
                if (beaconColor.senseColor() == ColorSense.Beacon_colors.red) {
                    telemetry.addData("Beacon Color", "Red");
                    robotDriver.go(RobotDriver.Speed.speed3, -10);  // move back
                    robotDriver.turnToAngle(0, 5); // turn a little to hit the button on left
                    robotDriver.go(RobotDriver.Speed.speed3, 10);  // run into the button
                } else if (beaconColor.senseColor() == ColorSense.Beacon_colors.blue) {
                    telemetry.addData("Beacon Color", "Blue");
                    robotDriver.go(RobotDriver.Speed.speed3, -10);  // move back
                    robotDriver.turnToAngle(0, -5); // turn a little to hit the button on right
                    robotDriver.go(RobotDriver.Speed.speed3, 10);  // run into the button
                } else {
                    telemetry.addData("Beacon Color", "Undetected");
                }
                */

                // Blue alliance
                /*
                if (beaconColor.senseColor() == ColorSense.Beacon_colors.blue) {
                    robotDriver.go(RobotDriver.Speed.speed3, -10);  // move back
                    robotDriver.turnToAngle(0, 5); // turn a little to hit the button on left
                    robotDriver.go(RobotDriver.Speed.speed3, 10);  // run into the button
                } else if (beaconColor.senseColor() == ColorSense.Beacon_colors.red) {
                    robotDriver.go(RobotDriver.Speed.speed3, -10);  // move back
                    robotDriver.turnToAngle(0, -5); // turn a little to hit the button on right
                    robotDriver.go(RobotDriver.Speed.speed3, 10);  // run into the button
                } else {
                    telemetry.addData("Beacon Color", "Undetected");
                }
                */

                telemetry.update();
            }

        } else {  // Vuforia not visible, go after the cap ball or toward the corner ramp
            telemetry.addData("Vuforia", "NOT visible");
        }
        telemetry.update();


        // now park the robot either to the ramp of the center vortex
        // fill in code here


        while(opModeIsActive()) {
            // to avoid crashes
            idle();
        }
    }
}
