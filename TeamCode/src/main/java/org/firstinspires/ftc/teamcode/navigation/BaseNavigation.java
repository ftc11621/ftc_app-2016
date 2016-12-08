package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.core.ButtonPusher;
import org.firstinspires.ftc.teamcode.core.ColorSense;
import org.firstinspires.ftc.teamcode.core.Launcher;
import org.firstinspires.ftc.teamcode.core.ParticleDoor;
import org.firstinspires.ftc.teamcode.core.RobotDriver;
import org.firstinspires.ftc.teamcode.core.VuforiaSensor;

/**
 * Created by HeavenDog on 11/19/2016.
 */

public abstract class BaseNavigation extends LinearOpMode {

    RobotDriver robotDriver;
    VuforiaSensor vuforia;
    Launcher launcher;
    ButtonPusher buttonPusher;
    ElapsedTime runtime= new ElapsedTime();

    // Coordinates of the Vuforia pictures
    final float Vuforia_wheels_x = 12 * 25.4f;
    final float Vuforia_wheels_y = (12 * 12 - 2) * 25.4f / 2.0f;
    final float Vuforia_legos_x = -36 * 25.4f;
    final float Vuforia_legos_y = (12 * 12 - 2) * 25.4f / 2.0f;
    final float Vuforia_tools_x = -(12 * 12 - 2) * 25.4f / 2.0f;
    final float Vuforia_tools_y = 36 * 25.4f;
    final float Vuforia_gears_x = -(12 * 12 - 2) * 25.4f / 2.0f;
    final float Vuforia_gears_y = -12 * 25.4f;


    @Override public void runOpMode() {

        vuforia = new VuforiaSensor(true);
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
        while(runtime.seconds() < 5.0 && !vuforia.updateRobotLocation()) {

            double toAngle = vuforia.getRobotNeedToTurnAngle(destination_x, destination_y);


            runtime.reset();
            while(runtime.seconds() < 5.0 && Math.abs(toAngle) > 45){ // if the camera does not face the picture, incorrect location
                sleep(500);
                vuforia.updateRobotLocation();
                toAngle = vuforia.getRobotNeedToTurnAngle(destination_x, destination_y);
            }

            double distance_CM = 0.1 * vuforia.getDestinationDistance(destination_x, destination_y); // in CM

            telemetry.addData("X" , vuforia.getX());
            telemetry.addData("Y", vuforia.getY());
            telemetry.addData("Distance to Gears", "%.0f", vuforia.getDestinationDistance(Vuforia_gears_x,Vuforia_gears_y));
            telemetry.addData("Angle to Gears", "%.0f", vuforia.getRobotNeedToTurnAngle(Vuforia_gears_x,Vuforia_gears_y));
            telemetry.update();

            robotDriver.setSpeed(RobotDriver.Speed.speed4);
            robotDriver.turnToAngle(0, toAngle);
            robotDriver.go(RobotDriver.Speed.speed3, distance_CM);
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
        robotDriver.go(RobotDriver.Speed.speed3, distance_to_move); // negative for intake front

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
