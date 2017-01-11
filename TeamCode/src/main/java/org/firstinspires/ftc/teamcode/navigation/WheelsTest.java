package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.BeaconColor;
import org.firstinspires.ftc.teamcode.core.ColorSense;
import org.firstinspires.ftc.teamcode.core.Picture;
import org.firstinspires.ftc.teamcode.core.Speed;
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

        sleep(1000);
        moveToPosition(Picture.wheels.getX()  , Picture.wheels.getY() - 120);
        ColorSense colorSense = new ColorSense(hardwareMap);
        robotDriver.turnToAngle(0,-15);

        if(BeaconColor.red.equals(colorSense.senseColor())){
            robotDriver.go(Speed.speed2, 10);
        } else {
            robotDriver.turnToAngle(0,30);
            robotDriver.go(Speed.speed2, 10);
        }

        vuforia.telemetryUpdate(telemetry);

        sleep(10000);
    }
}
