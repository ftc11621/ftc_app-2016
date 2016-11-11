package org.firstinspires.ftc.teamcode;

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


public class VuforiaNav {

    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    public static final String TAG = "Vuforia Sample";
    OpenGLMatrix lastRobotLocation = null;
    VuforiaLocalizer vuforia;
    VuforiaTrackables targets = null;
    VuforiaTrackable wheels = null;
    VuforiaTrackable legos = null;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    // Constructor
    public VuforiaNav (boolean  BlueOrRed) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdksQ3j/////AAAAGVB9GUsSEE0BlMaVB7HcRZRM4Sv74bxusFbCpn3gwnUkr3GuOtSWhrTCHnTU/93+Im+JlrYI6///bytu1igZT48xQ6182nSTpVzJ2ZP+Q/sNzSg3qvIOMnjEptutngqB+e3mQ1+YTiDa9aZod1e8X7UvGsAJ3cfV+X/S3E4M/81d1IRSMPRPEaLpKFdMqN3AcbDpBHoqp82fAp7XWVN3qd/BRe0CAAoNsr26scPBAxvm9cizRG1WeRSFms3XkwFN6eGpH7VpNAdPPXep9RQ3lLZMTFQGOfiV/vRQXq/Tlaj/b7dkA12zBSW81MfBiXRxp06NGieFe7KvXNuu2aDyyXoaPFsI44FEGp1z/SVSEVR4"; // Insert your own key here
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        targets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        wheels = targets.get(0);
        wheels.setName("wheels");

        legos  = targets.get(2);
        legos.setName("legos");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        //List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);
        OpenGLMatrix wheelsLocationOnField = OpenGLMatrix
                .translation(12*mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(wheelsLocationOnField);
        RobotLog.ii(TAG, "Wheels Target=%s", format(wheelsLocationOnField));


        OpenGLMatrix legosLocationOnField = OpenGLMatrix
                .translation(-36*mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(legosLocationOnField);
        RobotLog.ii(TAG, "Legos Target=%s", format(legosLocationOnField));

        // for phone in front
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 90, 0));  // -90,0,0 for the right side
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));


        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

    }

    public boolean isWheel_visible() {
        return ((VuforiaTrackableDefaultListener) wheels.getListener()).isVisible();
    }
    public boolean isLego_visible() {
        return ((VuforiaTrackableDefaultListener) legos.getListener()).isVisible();
    }

    public boolean updateRobotLocation()  {
        OpenGLMatrix robotLocationTransform = null;
        boolean currentlocation_flag = false;
        for (VuforiaTrackable trackable : allTrackables) {

            robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastRobotLocation = robotLocationTransform;
                currentlocation_flag = true;
            }
        }
        return currentlocation_flag;       // when no new location found
    }

    public double getX() {
        float[] coordinates = lastRobotLocation.getTranslation().getData();
        return coordinates[0];
    }
    public double getY() {
        float[] coordinates = lastRobotLocation.getTranslation().getData();
        return coordinates[1];
    }

    public double getDestinationDistance(double destination_X, double destination_Y) {
        return Math.sqrt((getX()-destination_X)*(getX()-destination_X) + (getY()-destination_Y)*(getY()-destination_Y));
    }

    public float getOrientation(int angleorder) {  // 1st, 2nd, and 3rd angle
        float orient_angle;
        switch (angleorder) {
            case 1:
                orient_angle = Orientation.getOrientation(lastRobotLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
                break;
            case 2:
                orient_angle = Orientation.getOrientation(lastRobotLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
                break;
            default:
                orient_angle = Orientation.getOrientation(lastRobotLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        }
        return orient_angle;
    }

    // angle > 0 when the destination is on the right side of the robot
    public double get_robot_need_to_turn_Angle(double destination_X, double destination_Y) {
        double destination_from_y_axis_angle = Math.toDegrees( Math.atan2(destination_X-getX(), destination_Y-getY()));
        return  destination_from_y_axis_angle + getOrientation(3);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}
