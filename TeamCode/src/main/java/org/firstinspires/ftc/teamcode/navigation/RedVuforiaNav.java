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

        Speed speed = Speed.speed7;

        moveAndShoot(38);
        sleep(800);
        robotDriver.turnToAngle(0,180);


        moveToPosition(Picture.gears.getX(24)  , Picture.gears.getY(24), speed);


        sleep(800);
        moveToPosition(Picture.gears.getX(6)  , Picture.gears.getY(6), speed);


        pushBeacon(BeaconColor.red);

        vuforia.telemetryUpdate(telemetry);

        sleep(10000);
    }

}
