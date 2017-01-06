package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.ButtonPusher;
import org.firstinspires.ftc.teamcode.core.Launcher;
import org.firstinspires.ftc.teamcode.core.ParticleDoor;
import org.firstinspires.ftc.teamcode.core.Picture;
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
    protected ElapsedTime runtime= new ElapsedTime();




    @Override public void runOpMode() {

        vuforia = new VuforiaSensor();
        robotDriver = new RobotDriver(hardwareMap);
        launcher = new Launcher(hardwareMap);
        buttonPusher = new ButtonPusher(hardwareMap);

        //ElapsedTime runtime = new ElapsedTime();



        baseLog(">", "Press Play to start tracking");
        waitForStart();
        vuforia.activate();
        navigate();







    }

    protected void baseLog(String key, String messasge) {
        telemetry.addData(key, messasge);
        telemetry.update();
    }

    protected abstract void navigate();

    public boolean moveToPosition(double destination_x, double destination_y) {   // in mm
        runtime.reset();

        while(runtime.seconds()< 5.0 && !vuforia.isWheel_visible() && !vuforia.isLego_visible() && !vuforia.isTools_visible() && !vuforia.isGears_visible()) { // 5 sec timeout to find a blue pattern
            telemetry.addData("Vuforia", "NOT visible");
            telemetry.update();
        }

        if (!vuforia.isWheel_visible() && !vuforia.isLego_visible() && !vuforia.isTools_visible() && !vuforia.isGears_visible()) {
            return false;
        }

        telemetry.addData("Vuforia picture", "Visible");
        telemetry.update();

        runtime.reset();    // 5 seconds timeout if it can't find location
//        while(runtime.seconds() < 5.0 && !vuforia.updateRobotLocation()) {
        if(vuforia.updateRobotLocation()) {
            vuforia.telemetryUpdate(telemetry);
            double toAngle = vuforia.getRobotNeedToTurnAngle(destination_x, destination_y);


            runtime.reset();
            while(runtime.seconds() < 5.0 && Math.abs(toAngle) > 45){ // if the camera does not face the picture, incorrect location
                sleep(500);
                vuforia.updateRobotLocation();
                toAngle = vuforia.getRobotNeedToTurnAngle(destination_x, destination_y);
            }

            double distance_CM = 0.1 * vuforia.getDestinationDistance(destination_x, destination_y); // in CM
            telemetry.addData("DistanceToTargetCM", distance_CM/2.54);
            vuforia.telemetryUpdate(telemetry);

            robotDriver.setSpeed(Speed.speed4);
            robotDriver.turnToAngle(0, toAngle);
            robotDriver.go(Speed.speed3, distance_CM);
            sleep(500);
        }
        if(runtime.seconds()>=5.0) { // fail to update location by timeout
            return false;
        }
        //stop();
        return true;
    }

    // to move a distance then shoot two particles
    public void moveAndShoot(double distance_to_move) {
        ParticleDoor partDoor = new ParticleDoor(hardwareMap); // on top to make sure it opens before interrupt
        robotDriver.go(Speed.speed3, distance_to_move * InchesToCentimeters); // negative for intake front

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

}
