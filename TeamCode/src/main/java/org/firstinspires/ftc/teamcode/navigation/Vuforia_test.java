package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.ParticleDoor;
import org.firstinspires.ftc.teamcode.core.RobotDriver;

/**
 * Created by HeavenDog on 12/4/2016.
 */

@Autonomous(name="Vuforia Test", group ="Test")
//@Disabled
public class Vuforia_test extends BaseNavigation {


    public void navigate() {

        ElapsedTime runtime_navigate = new ElapsedTime();

        while (opModeIsActive()) {
            vuforia.telemetryUpdate(telemetry);
            idle();
        }
    }
}
