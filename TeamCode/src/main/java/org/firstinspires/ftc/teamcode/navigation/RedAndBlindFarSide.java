package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.Speed;
import org.firstinspires.ftc.teamcode.core.Turn;

/**
 * Created by Marie on 12/8/2016.
 */
@Autonomous(name="Red Blind Ramp Far Side", group ="Competition")
//@Disabled
public class RedAndBlindFarSide extends BaseNavigation{
    @Override
    protected void navigate() {
        sleep(10000);
        moveAndShoot(-54);
        robotDriver.turnToAngle(0,120);
        robotDriver.go(Speed.speed4,35 * InchesToCentimeters);



        while(opModeIsActive()) {       // to prevent stuck in a loop
            // to avoid crashes
            idle();
        }
    }
}
