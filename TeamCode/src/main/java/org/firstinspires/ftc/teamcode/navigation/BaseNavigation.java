package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.BeaconColor;
import org.firstinspires.ftc.teamcode.core.ButtonPusher;
import org.firstinspires.ftc.teamcode.core.ColorSense;
import org.firstinspires.ftc.teamcode.core.Launcher;
import org.firstinspires.ftc.teamcode.core.ParticleDoor;
import org.firstinspires.ftc.teamcode.core.RobotDriver;
import org.firstinspires.ftc.teamcode.core.Speed;
import org.firstinspires.ftc.teamcode.core.VuforiaSensor;

/**
 * Created by HeavenDog on 11/19/2016.
 */

public abstract class BaseNavigation extends LinearOpMode {

    protected final double InchesToCentimeters = 2.54;
    protected RobotDriver robotDriver;
    protected VuforiaSensor vuforia;
    protected Launcher launcher;
    protected ButtonPusher buttonPusher;
    protected ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        vuforia = new VuforiaSensor();
        robotDriver = new RobotDriver(hardwareMap, this);
        launcher = new Launcher(hardwareMap);
        buttonPusher = new ButtonPusher(hardwareMap);


        baseLog(">", "Press Play to start tracking");
        waitForStart();
        vuforia.activate();
        navigate();


    }

    protected void baseLog(String key, String messasge) {
        telemetry.addData(key, messasge);
        telemetry.update();
    }


    protected void findPicture()
    {
        int totalTurn = 0;
        while(!aPictureIsVisible())
        {
            robotDriver.turnToAngle(15);
            totalTurn += 15;
            if(totalTurn >= 360) break;
            sleep(500);
        }
    }

    protected abstract void navigate();

    public boolean moveToPosition(double destination_x, double destination_y, Speed speed) {   // in mm
        runtime.reset();


        if (!aPictureIsVisible()) {
            telemetry.addData("Vuforia", "NOT visible");
            telemetry.update();
            return false;
        }

        telemetry.addData("Vuforia picture", "Visible");
        telemetry.update();

        runtime.reset();    // 5 seconds timeout if it can't find location
        robotDriver.setSpeed(speed);
        //while(runtime.seconds() < 15.0) {
        if (vuforia.updateRobotLocation()) {

            turnToPoint(destination_x, destination_y);
            turnToPoint(destination_x, destination_y);
            turnToPoint(destination_x, destination_y);

            //Turn to desired angle
            //check the angle to see how close it is to to the disired angle


            double distance_CM = 0.1 * vuforia.getDestinationDistance(destination_x, destination_y); // in CM
            telemetry.addData("DistanceToTargetCM", distance_CM / 2.54);
            sleep(500);
            //vuforia.telemetryUpdate(telemetry);
            robotDriver.go(speed, distance_CM);
        }


        //if(Math.abs(xDistance) < 250 && Math.abs(yDistance) < 250) break;
        //}
        /*if(runtime.seconds()>=5.0) { // fail to update location by timeout
            return false;
        }*/

        return true;
    }

    private double turnToPoint(double destination_x, double destination_y) {
        double toAngle = vuforia.getRobotNeedToTurnAngle(destination_x, destination_y);
        robotDriver.turnToAngle(0, toAngle);
        sleep(75);
        vuforia.updateRobotLocation();
        return toAngle;
    }

    private boolean aPictureIsVisible() {
        return vuforia.isWheel_visible() || vuforia.isLego_visible() || vuforia.isTools_visible() || vuforia.isGears_visible();
    }

    // to move a distance then shoot two particles
    public void moveAndShoot(double distance_to_move) {
        ParticleDoor partDoor = new ParticleDoor(hardwareMap); // on top to make sure it opens before interrupt
        robotDriver.go(Speed.speed7, distance_to_move * InchesToCentimeters); // negative for intake front

        launcher.shoot();   // shoot 1st particle
        partDoor.openDoor();
        runtime.reset();
        while (runtime.seconds() < 1.0) {    // wait for the chance the door is fully open
        }
        launcher.shoot();   // shoot 2nd particle
        partDoor.closeDoor();  // to close the door at the end
        runtime.reset();
        while (runtime.seconds() < 1.0) {    // wait for the chance the door to close
        }
    }

    protected void pushBeacon(BeaconColor beaconColor) {
        ColorSense colorSense = new ColorSense(hardwareMap);

        /*if (BeaconColor.neither.equals(colorSense.senseColor())) {
            robotDriver.turnToAngle();
        }*/
        int anglePush = 0;
        if (!beaconColor.equals(colorSense.senseColor())) {
            robotDriver.turnToAngle(0, -12);
            anglePush = 12;
            robotDriver.go(Speed.speed2, 10);
        } else {
            robotDriver.turnToAngle(0, 12);
            anglePush = - 12;
            robotDriver.go(Speed.speed2, 10);
        }
        sleep(1000);
        if (!beaconColor.equals(colorSense.senseColor())) {
            robotDriver.go(Speed.speed3, -5);
            robotDriver.go(Speed.speed3, 5);
        }
        robotDriver.go(Speed.speed3, -10);
        robotDriver.turnToAngle(0, anglePush);
    }

}
