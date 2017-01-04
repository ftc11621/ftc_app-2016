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
        sleep(300);
        moveToPosition(Picture.wheels.getX() + 5 , Picture.wheels.getY() -20); // in inches

        sleep(1000);

        moveToPosition(Picture.wheels.getX() , Picture.wheels.getY() - 1);

        sleep(1000);

        moveToPosition(Picture.wheels.getX() - 5 , Picture.wheels.getY() - 10);

        sleep(1000);

        vuforia.telemetryUpdate(telemetry);
    }
}
