package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.ColorSense;

/**
 * Created by Marie on 1/3/2017.
 */
@Autonomous(name="color Test", group ="Test")
public class ColorSensorTest extends BaseNavigation {

    public void navigate() {

        ColorSense colorSense = new ColorSense(hardwareMap);
        ElapsedTime runtime_navigate = new ElapsedTime();

        while (opModeIsActive()) {
            colorSense.telemetryUpdate(telemetry);
            idle();
        }
    }
}
