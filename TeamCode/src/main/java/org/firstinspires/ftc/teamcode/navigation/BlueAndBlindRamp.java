package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.Speed;
import org.firstinspires.ftc.teamcode.core.Turn;

/**
 * Created by Marie on 12/8/2016.
 */
@Autonomous(name="Blue Blind Ramp 1", group ="Competition")

public class BlueAndBlindRamp extends BaseNavigation{
    @Override
    protected void navigate() {
        moveAndShoot(20);
        robotDriver.turn(Turn.right90);
        robotDriver.go(Speed.speed3,25 * InchesToCentimeters);

        while(opModeIsActive()) {       // to prevent stuck in a loop
            // to avoid crashes
            idle();
        }
    }
}
