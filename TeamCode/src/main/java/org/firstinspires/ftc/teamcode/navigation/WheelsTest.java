package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.Picture;
import org.firstinspires.ftc.teamcode.navigation.BaseNavigation;

/**
 * Created by Marie on 1/3/2017.
 */
@Autonomous(name="Wheels Test", group ="Test")
public class WheelsTest extends BaseNavigation{

    @Override
    protected void navigate() {


        sleep(1000);

        moveToPosition(Picture.wheels.getX()  , Picture.wheels.getY() - 500);
        moveToPosition(Picture.wheels.getX()  , Picture.wheels.getY() - 120);



        vuforia.telemetryUpdate(telemetry);

        sleep(10000);
    }
}
