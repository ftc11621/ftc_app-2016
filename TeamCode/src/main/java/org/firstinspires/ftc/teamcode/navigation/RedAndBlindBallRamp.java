package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.Speed;

/**
 * Created by Marie on 12/8/2016.
 */
@Autonomous(name="Red Blind Ball Ramp 1", group ="Competition")

public class RedAndBlindBallRamp extends BaseNavigation{
    @Override
    protected void navigate() {
        moveAndShoot(-33);      // move and shoot
        robotDriver.turnToAngle(0,70); // turn toward the ball
        robotDriver.go(Speed.speed4,-24 * InchesToCentimeters);     // move close to the ball
        robotDriver.turnToAngle(0,40);      // turn toward the left side of the cap ball
        robotDriver.turnToAngle(0,-30);      // turn toward the left side of the cap ball
        //robotDriver.go(Speed.speed4,40 * InchesToCentimeters);  // hit the cap ball
        //robotDriver.turnToAngle(0,-70);     // turn left
        robotDriver.go(Speed.speed4,50 * InchesToCentimeters); // go toward the ramp


        while(opModeIsActive()) {       // to prevent stuck in a loop
            // to avoid crashes
            idle();
        }
    }
}
