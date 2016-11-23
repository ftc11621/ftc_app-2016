package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.ButtonPusher;
import org.firstinspires.ftc.teamcode.core.Launcher;
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



    @Override public void runOpMode() {

        vuforia = new VuforiaSensor(true);
        robotDriver = new RobotDriver(hardwareMap);
        launcher = new Launcher(hardwareMap);
        buttonPusher = new ButtonPusher(hardwareMap);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        vuforia.activate();
        navigate();




    }

    protected abstract void navigate();

    public void moveToPosition(double destination_x, double destination_y) {
        double distance = vuforia.getDestinationDistance(destination_x, destination_y);
        double toAngle = vuforia.getRobotNeedToTurnAngle(destination_x, destination_y);
        double fromAngle = vuforia.getOrientation(3);
        robotDriver.turnToAngle(fromAngle,toAngle);
        robotDriver.go(RobotDriver.Speed.normal, distance);
        stop();
    }

}
