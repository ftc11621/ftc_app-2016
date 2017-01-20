package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.BeaconColor;
import org.firstinspires.ftc.teamcode.core.Picture;
import org.firstinspires.ftc.teamcode.core.Speed;

/**
 * Created by Marie on 1/3/2017.
 */
@Autonomous(name="RedVuforia", group ="Competition")
public class RedVuforiaNav extends BaseNavigation{

    @Override
    protected void navigate() {

        Speed speed = Speed.speed3;

        moveAndShoot(-38);
        sleep(800);
        robotDriver.turnToAngle(0,180);

        robotDriver.turnToAngle(0,45);
        robotDriver.go(speed, 24 * 2.5 );
        robotDriver.turnToAngle(0,-90);
        sleep(2000);
        moveToPosition(Picture.gears.getX(24)  , Picture.gears.getY(24), speed);



        moveToPosition(Picture.gears.getX(6)  , Picture.gears.getY(6), speed);


        pushBeacon(BeaconColor.red);
        robotDriver.go(speed, -20 * 2.5);

        robotDriver.turnToAngle(0, 95);
        robotDriver.go(speed, 48 * 2.5);
        robotDriver.turnToAngle(0,-90);
        moveToPosition(Picture.tools.getX(6)  , Picture.tools.getY(6), speed);
        pushBeacon(BeaconColor.red);
        vuforia.telemetryUpdate(telemetry);

        sleep(10000);
    }

}
