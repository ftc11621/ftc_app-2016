package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.BeaconColor;
import org.firstinspires.ftc.teamcode.core.Picture;
import org.firstinspires.ftc.teamcode.core.Speed;

/**
 * Created by Marie on 1/3/2017.
 */
@Autonomous(name="2 Beacon Wheels Test", group ="Test")
public class WheelsTwoBeaconTest extends BaseNavigation{

    @Override
    protected void navigate() {

        Speed speed = Speed.speed3;
        sleep(1000);

        //findPicture();

        moveToPosition(Picture.wheels.getX(24)  , Picture.wheels.getY(24), speed);

        //findPicture();
        moveToPosition(Picture.wheels.getX(6)  , Picture.wheels.getY(6), speed);


        pushBeacon(BeaconColor.blue);

        robotDriver.go(speed, -20 * 2.5);

        robotDriver.turnToAngle(0, -95);
        robotDriver.go(speed, 48 * 2.5);
        robotDriver.turnToAngle(0,90);
        moveToPosition(Picture.legos.getX(6)  , Picture.legos.getY(6), speed);

        vuforia.telemetryUpdate(telemetry);

        sleep(10000);
    }

}
