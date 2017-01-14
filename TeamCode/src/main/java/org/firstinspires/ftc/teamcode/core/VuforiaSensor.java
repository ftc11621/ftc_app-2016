package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

@Autonomous(name="Vuforia", group ="Competition")
public class VuforiaSensor {

    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    public static final String TAG = "Vuforia Sample";
    OpenGLMatrix lastRobotLocation = null;
    VuforiaLocalizer vuforia;
    VuforiaTrackables targets = null;
    VuforiaTrackable wheels = null;
    VuforiaTrackable legos = null;
    VuforiaTrackable tools = null;
    VuforiaTrackable gears = null;


    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    // Constructor
    public VuforiaSensor() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdksQ3j/////AAAAGVB9GUsSEE0BlMaVB7HcRZRM4Sv74bxusFbCpn3gwnUkr3GuOtSWhrTCHnTU/93+Im+JlrYI6///bytu1igZT48xQ6182nSTpVzJ2ZP+Q/sNzSg3qvIOMnjEptutngqB+e3mQ1+YTiDa9aZod1e8X7UvGsAJ3cfV+X/S3E4M/81d1IRSMPRPEaLpKFdMqN3AcbDpBHoqp82fAp7XWVN3qd/BRe0CAAoNsr26scPBAxvm9cizRG1WeRSFms3XkwFN6eGpH7VpNAdPPXep9RQ3lLZMTFQGOfiV/vRQXq/Tlaj/b7dkA12zBSW81MfBiXRxp06NGieFe7KvXNuu2aDyyXoaPFsI44FEGp1z/SVSEVR4"; // Insert your own key here
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        targets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        wheels = targets.get(0);
        wheels.setName("wheels");

        tools = targets.get(1);
        tools.setName("tools");

        legos  = targets.get(2);
        legos.setName("legos");

        gears  = targets.get(3);
        gears.setName("gears");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        //List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        OpenGLMatrix wheelsLocationOnField = OpenGLMatrix
                .translation(12*mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));


        OpenGLMatrix legosLocationOnField = OpenGLMatrix
                .translation(-36*mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));


        OpenGLMatrix toolsLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, 36*mmPerInch, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));


        OpenGLMatrix gearsLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, -12*mmPerInch, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));


        // for phone in front, 6mm to the left
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(-50, 0, 30)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 90, 180));  // insert phone from the left front
                        //AngleUnit.DEGREES, -90, 90, 0));  // -90,0,0 for the right side

        wheels.setLocation(wheelsLocationOnField);
        legos.setLocation(legosLocationOnField);
        tools.setLocation(toolsLocationOnField);
        gears.setLocation(gearsLocationOnField);

        RobotLog.ii(TAG, "Wheels Target=%s", format(wheelsLocationOnField));
        RobotLog.ii(TAG, "Legos Target=%s", format(legosLocationOnField));
        RobotLog.ii(TAG, "Tools Target=%s", format(toolsLocationOnField));
        RobotLog.ii(TAG, "Gears Target=%s", format(gearsLocationOnField));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

    }

    public boolean isWheel_visible() {
        return ((VuforiaTrackableDefaultListener) wheels.getListener()).isVisible();
    }
    public boolean isLego_visible() {
        return ((VuforiaTrackableDefaultListener) legos.getListener()).isVisible();
    }

    public boolean isTools_visible() {
        return ((VuforiaTrackableDefaultListener) tools.getListener()).isVisible();
    }
    public boolean isGears_visible() {
        return ((VuforiaTrackableDefaultListener) gears.getListener()).isVisible();
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

    public void getLocation()
    {

    }

    public double getDistanceToPicture(Picture picture){
        return getDestinationDistance(picture.getX(), picture.getY());
    }

    public double getAngleToPicture(Picture picture){
        return getRobotNeedToTurnAngle(picture.getX(), picture.getY());
    }

    public void telemetryUpdate(Telemetry telemetry){
        if(updateRobotLocation()) {
            telemetry.addData("X", "%.0f", getX());
            telemetry.addData("Y", "%.0f", getY());

            for (Picture picture : Picture.values()) {
                telemetry.addData("Distance to "+picture.name(), "%.0f inch", getDistanceToPicture(picture) / 25.4);
                telemetry.addData("Angle to "+picture.name(), "%.0f", getAngleToPicture(picture));
            }

            telemetry.update();
        }
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
        return Math.sqrt(Math.pow(getX()-destination_X,2) + Math.pow(getY()-destination_Y,2));
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
    public double getRobotNeedToTurnAngle(double destination_X, double destination_Y) {
        double destination_from_y_axis_angle = Math.toDegrees( Math.atan2(destination_X-getX(), destination_Y-getY()));
        return  destination_from_y_axis_angle + getOrientation(3);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public void activate() {
        this.targets.activate();
    }
}
