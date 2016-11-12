package org.firstinspires.ftc.teamcode.reference.camera;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;




@TeleOp(name="Concept: Driver", group = "Testing")
//@Disabled
public class driverPeriod extends OpMode {


    DcMotorController controllerFR;
    DcMotorController controllerI;


    int motorBR = 1;
    int motorBL = 2;
    int motorIntake = 1;
    int motorGConveyor = 2;




    VuforiaSensor vuforia = new VuforiaSensor(0,1,0);


    public driverPeriod() {
    }


    @Override
    public void init() {
        controllerFR = hardwareMap.dcMotorController.get("controller1");
        controllerI = hardwareMap.dcMotorController.get("controller2");


        vuforia.setupVuforia();
        vuforia.visionActiavte();


    }


    @Override
    public void loop() {




        //Left Motors
        float left = gamepad1.left_stick_y;


        left = Range.clip(left, -1, 1);


        left = (float) scaleInput(left);


        //Right Motors
        float right = -gamepad1.right_stick_y;


        right = Range.clip(right, -1, 1);


        right = (float) scaleInput(right);


        //Motor Powers
        controllerFR.setMotorPower(motorBR, -right);
        controllerFR.setMotorPower(motorBL, -left);




        if (gamepad1.right_trigger > 0.0) {
            controllerI.setMotorPower(motorIntake, 1.0);
        }
        else if(gamepad1.left_trigger > 0.0)
        {
            controllerI.setMotorPower(motorIntake, -1.0);
        }
        else
        {
            controllerI.setMotorPower(motorIntake, 0.0);
        }


        //telemetry for robot AND PICTURE PLACEMENT, DOES NOTHING
        if (vuforia.isVis("Gears Target")){
            telemetry.addData("Robot Location", "{"+vuforia.getRobot("Gears Target","x")+","+vuforia.getRobot("Tools Target","y")+"}");
            telemetry.addData("Robot Rotation", vuforia.getRobot("Gears Target","angle"));
            telemetry.addData("X and Y from Gears", "{"+vuforia.getRobot("Gears Target","xAway")+","+vuforia.getRobot("Tools Target","yAway")+"}");
        }
        if (vuforia.isVis("Tools Target")){
            telemetry.addData("Robot Location", "{"+vuforia.getRobot("Tools Target","x")+","+vuforia.getRobot("Tools Target","y")+"}");
            telemetry.addData("Robot Rotation", vuforia.getRobot("Tools Target","angle"));
            telemetry.addData("X and Y from Tools", "{"+vuforia.getRobot("Tools Target","xAway")+","+vuforia.getRobot("Tools Target","yAway")+"}");
        }


        telemetry.update();


        telemetry.addData("1 Text", "*** Robot Data***");
        telemetry.addData("2 Motor Power Left", String.format("%.2f", left));
        telemetry.addData("3 Motor Power Right", String.format("%.2f", right));


    }




    @Override
    public void stop() {


    }


    private double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };


        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }


        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }


        return dScale;
    }


}











