package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.ParticleDoor;
import org.firstinspires.ftc.teamcode.core.RobotDriver;

/**
 * Created by HeavenDog on 12/4/2016.
 */

@Autonomous(name="Autonomous Red Blind 1", group ="Competition")
//@Disabled
public class RedAndBlind extends BaseNavigation {




    public void navigate() {
        ElapsedTime runtime = new ElapsedTime();
        ParticleDoor pDoor = new ParticleDoor(hardwareMap);

        idle();

        // Wait for the game to start (driver presses PLAY)
        pDoor.closeDoor();
        waitForStart();

        //robotDriver.go(RobotDriver.Speed.normal, 30*2.54);

        launcher.shoot();

        pDoor.openDoor();
        runtime.reset();
        while(runtime.seconds() < 1.0) {    // wait for the chance the door is fully open
        }

        launcher.shoot();


        /*robotDriver.turnToAngle(90.0, 135.0);
        robotDriver.go(RobotDriver.Speed.normal, 36*2.54);
        robotDriver.turn(RobotDriver.Turn.right90);
        robotDriver.go(RobotDriver.Speed.normal, 38*2.54);*/

        //pDoor.closeDoor();  // helps to close for the next autonomous test so it stays close

    }
}
