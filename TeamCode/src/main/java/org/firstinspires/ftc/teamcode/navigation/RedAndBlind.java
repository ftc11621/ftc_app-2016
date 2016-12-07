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


        robotDriver.go(RobotDriver.Speed.speed3, -10 * 2.54); // negative for intake front

        launcher.shoot();   // shoot 1st particle
        pDoor.openDoor();
        runtime_navigate.reset();
        while (runtime_navigate.seconds() < 1.0) {    // wait for the chance the door is fully open
        }
        launcher.shoot();   // shoot 2nd particle

        /*

        robotDriver.turnToAngle(90.0, 135.0);
        robotDriver.go(RobotDriver.Speed.normal, 36*2.54);
        robotDriver.turn(RobotDriver.Turn.right90);
        robotDriver.go(RobotDriver.Speed.normal, 38*2.54);

        */


        //if (moveToPosition(12 * 25.4, (12 * 12 - 2) * 25.4 / 2.0 - 300.0)) { // 200 mm from wheels
        if (moveToPosition(Vuforia_gears_x + 300, Vuforia_gears_y)) { // 300 mm from gears
            telemetry.addData("Vuforia", "Visible");
        } else {
            telemetry.addData("Vuforia", "NOT visible");
        }
        telemetry.update();

        /*
        runtime_navigate.reset();
        while (runtime_navigate.seconds() < 5) {
            if(beaconColor.senseColor() == beaconColor.Beacon_colors.red) {
                telemetry.addData("Beacon Color", "Red");
            } else if(beaconColor.senseColor() == beaconColor.Beacon_colors.blue) {
                telemetry.addData("Beacon Color", "Blue");
            } else {
                telemetry.addData("Beacon Color", "Undetected");
            }
            telemetry.update();
        }
        */

        pDoor.closeDoor();  // to close the door at the end
    }
}
