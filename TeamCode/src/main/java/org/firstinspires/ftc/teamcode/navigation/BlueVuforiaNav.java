package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.BeaconColor;
import org.firstinspires.ftc.teamcode.core.Picture;
import org.firstinspires.ftc.teamcode.core.Speed;

/**
 * Created by Marie on 1/3/2017.
 */
@Autonomous(name="BlueVuforia", group ="Competition")
public class BlueVuforiaNav extends BaseNavigation{

    @Override
    protected void navigate() {

        Speed speed = Speed.speed3;

        //moveAndShoot(38);
        robotDriver.go(speed, 50 * 2.54);

        robotDriver.turnToAngle(0,-65);
        robotDriver.go(speed, 28 * 2.5 );
        robotDriver.turnToAngle(0,80);
        sleep(500);

        moveToPosition(Picture.wheels.getX(24)  , Picture.wheels.getY(24), speed);

        //findPicture();
        moveToPosition(Picture.wheels.getX(6)  , Picture.wheels.getY(6), speed);

        pushBeacon(BeaconColor.blue);

    }

}
