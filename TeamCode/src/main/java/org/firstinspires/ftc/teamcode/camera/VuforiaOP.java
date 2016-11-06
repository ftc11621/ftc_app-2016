package org.firstinspires.ftc.teamcode.camera;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.HINT;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;


/**
 * Created by abrah on 10/14/2016.
 */
public class VuforiaOP extends LinearOpMode {


    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AQOWMXn/////AAAAGWYPWNBbCk0gtnLrpRXAzO9bS+VnD/Wd9Pign89BeWodWrRXamhMi5AX7ErlnOnYEHKwCEssFuJ2dlBqsKia/2Q1H6dFeQUHPeesNoCnzO7oeU6dnIMrH+EQuWDNTtPBVq+OhQDmHRt+0XpPr8MI74+hixoupz/GdJyIQANn8Y/qwF2j9rm9RaU2+aS14vdLcwLXhVotRKU3IZrQeYOdU2nuskzlOKG+/oPUZJMdOhJI+ONVXRAsB19vf00e/anLybmlMu0wWDg3x3eg2WpyrBnKSLLgYO6wpL70Lcb/OI4XmQJxq251/x6dqdHMjDZq8JdGOdiGw0umm1Y0cX6K56b7LgzWpWMMwBGCY3pTRdAgAQOWMXn/////AAAAGWYPWNBbCk0gtnLrpRXAzO9bS+VnD/Wd9Pign89BeWodWrRXamhMi5AX7ErlnOnYEHKwCEssFuJ2dlBqsKia/2Q1H6dFeQUHPeesNoCnzO7oeU6dnIMrH+EQuWDNTtPBVq+OhQDmHRt+0XpPr8MI74+hixoupz/GdJyIQANn8Y/qwF2j9rm9RaU2+aS14vdLcwLXhVotRKU3IZrQeYOdU2nuskzlOKG+/oPUZJMdOhJI+ONVXRAsB19vf00e/anLybmlMu0wWDg3x3eg2WpyrBnKSLLgYO6wpL70Lcb/OI4XmQJxq251/x6dqdHMjDZq8JdGOdiGw0umm1Y0cX6K56b7LgzWpWMMwBGCY3pTRdAg"; //Enter Key Here;


        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;


        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);




        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");


        waitForStart();


        while (opModeIsActive()) {
            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();


                if (pose != null) {
                    VectorF translation = pose.getTranslation();


                    telemetry.addData(beac.getName() + "-Translation", translation);


                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2))); //vertical phone. for landscape phone translation.get(0), translation.get(2)


                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                }
            }
            telemetry.update();
        }
    }
}



