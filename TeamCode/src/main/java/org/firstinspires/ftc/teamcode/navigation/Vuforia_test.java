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
            if(vuforia.updateRobotLocation()) {
                telemetry.addData("X", "%.0f", vuforia.getX());
                telemetry.addData("Y", "%.0f", vuforia.getY());

                telemetry.addData("Distance to Gears", "%.0f", vuforia.getDestinationDistance(Vuforia_gears_x,Vuforia_gears_y)/25.4);
                telemetry.addData("Angle to Gears", "%.0f", vuforia.getRobotNeedToTurnAngle(Vuforia_gears_x,Vuforia_gears_y));

                telemetry.addData("Distance to Tools", "%.0f", vuforia.getDestinationDistance(Vuforia_tools_x,Vuforia_tools_y)/25.4);
                telemetry.addData("Angle to Tools", "%.0f", vuforia.getRobotNeedToTurnAngle(Vuforia_tools_x,Vuforia_tools_y));

                telemetry.addData("Distance to Wheels", "%.0f", vuforia.getDestinationDistance(Vuforia_wheels_x,Vuforia_wheels_y)/25.4);
                telemetry.addData("Angle to Wheels", "%.0f", vuforia.getRobotNeedToTurnAngle(Vuforia_wheels_x,Vuforia_wheels_y));

                telemetry.addData("Distance to Legos", "%.0f", vuforia.getDestinationDistance(Vuforia_legos_x,Vuforia_legos_y)/25.4);
                telemetry.addData("Angle to Legos", "%.0f", vuforia.getRobotNeedToTurnAngle(Vuforia_legos_x,Vuforia_legos_y));


                telemetry.update();
            }
            idle();
        }
    }
}
