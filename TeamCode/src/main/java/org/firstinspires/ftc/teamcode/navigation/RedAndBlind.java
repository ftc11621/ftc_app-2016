package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.ParticleDoor;
import org.firstinspires.ftc.teamcode.core.RobotDriver;

/**
 * Created by HeavenDog on 12/4/2016.
 */

@Autonomous(name="Autonomous when you're blind", group ="Competition")
//@Disabled
public class RedAndBlind extends BaseNavigation {
    public void navigate() {
        robotDriver.go(RobotDriver.Speed.normal, 30*2.54);
        ParticleDoor pDoor = new ParticleDoor(hardwareMap);
        launcher.shoot();
        pDoor.openDoor();
        launcher.shoot();
        /*robotDriver.turnToAngle(90.0, 135.0);
        robotDriver.go(RobotDriver.Speed.normal, 36*2.54);
        robotDriver.turn(RobotDriver.Turn.right90);
        robotDriver.go(RobotDriver.Speed.normal, 38*2.54);*/



    }
}
