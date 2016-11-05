package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;


public class Vuforia {

    public static final String TAG = "Vuforia Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
    VuforiaTrackables visionTargets;

    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    // phone location in robot, change if necessary
    // phone in front is 0, -90, 0 rotation
    OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(mmBotWidth/2,0,0)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.YZY,
                    AngleUnit.DEGREES, 0, -90, 0));  // -90, 0, 0 when on the right side


    // Constructor
    public Vuforia(){
        visionTargets = this.vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        // get the license key from the Vuforia website, you may need to register first
        parameters.vuforiaLicenseKey = "AQOWMXn/////AAAAGWYPWNBbCk0gtnLrpRXAzO9bS+VnD/Wd9Pign89BeWodWrRXamhMi5AX7ErlnOnYEHKwCEssFuJ2dlBqsKia/2Q1H6dFeQUHPeesNoCnzO7oeU6dnIMrH+EQuWDNTtPBVq+OhQDmHRt+0XpPr8MI74+hixoupz/GdJyIQANn8Y/qwF2j9rm9RaU2+aS14vdLcwLXhVotRKU3IZrQeYOdU2nuskzlOKG+/oPUZJMdOhJI+ONVXRAsB19vf00e/anLybmlMu0wWDg3x3eg2WpyrBnKSLLgYO6wpL70Lcb/OI4XmQJxq251/x6dqdHMjDZq8JdGOdiGw0umm1Y0cX6K56b7LgzWpWMMwBGCY3pTRdAg";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);


        VuforiaTrackable wheels = visionTargets.get(0); wheels.setName("Wheels Target");
        VuforiaTrackable tools = visionTargets.get(1);  tools.setName("Tools Target");
        VuforiaTrackable legos = visionTargets.get(2);  legos.setName("Legos Target");
        VuforiaTrackable gears = visionTargets.get(3);  gears.setName("Gears Target");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        //List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        //allTrackables.addAll(visionTargets);

        // target locations on the field
        OpenGLMatrix tools_redTargetLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, 36*mmPerInch, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        OpenGLMatrix gears_redTargetLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, -12*mmPerInch, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        OpenGLMatrix wheels_blueTargetLocationOnField = OpenGLMatrix
                .translation(12*mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        OpenGLMatrix legos_blueTargetLocationOnField = OpenGLMatrix
                .translation(-36*mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));


        tools.setLocation(tools_redTargetLocationOnField);
        gears.setLocation(gears_redTargetLocationOnField);
        wheels.setLocation(wheels_blueTargetLocationOnField);
        legos.setLocation(legos_blueTargetLocationOnField);

       // wheels.setLocation(createMatrix(12*mmPerInch, mmFTCFieldWidth/2, 0, 90, 0, 0));  // blue wall
       // tools.setLocation(createMatrix(-mmFTCFieldWidth/2, 36*mmPerInch, 0, 90, 90, 0)); // red wall
       // legos.setLocation(createMatrix(-36*mmPerInch, mmFTCFieldWidth/2, 0, 90, 0, 0));  // blue wall
       // gears.setLocation(createMatrix(-mmFTCFieldWidth/2, -12*mmPerInch, 0, 90, 90, 0)); // red wall


        RobotLog.ii(TAG, "Tools Red wall Target=%s", format(tools_redTargetLocationOnField));
        RobotLog.ii(TAG, "Gears Red wall Target=%s", format(gears_redTargetLocationOnField));
        RobotLog.ii(TAG, "Wheels Blue wall Target=%s", format(wheels_blueTargetLocationOnField));
        RobotLog.ii(TAG, "Legos Blue wall Target=%s", format(wheels_blueTargetLocationOnField));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);


    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    /*
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }
    */

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}
