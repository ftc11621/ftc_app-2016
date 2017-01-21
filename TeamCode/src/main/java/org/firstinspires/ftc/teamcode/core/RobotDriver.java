package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class RobotDriver {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    private OpMode opMode;

    //private double timeoutS = 30;

    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_CM       = 9.15 ;     // For figuring circumference
    private static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * Math.PI );


    private static final double     WHEELS_SPACING_CM       = 40.8;     // spacing between wheels for turns

    private ElapsedTime runtime = new ElapsedTime();
    private int MAX_TIMEOUT = 2;

    public RobotDriver(HardwareMap hardwareMap, OpMode opMode) {
        this.leftMotor  = hardwareMap.dcMotor.get("motor_2");
        this.rightMotor = hardwareMap.dcMotor.get("motor_1");
        this.opMode = opMode;
        forward();
    }

    public void setRunMode(DcMotor.RunMode runMode){
        leftMotor.setMode(runMode);
        rightMotor.setMode(runMode);
    }

    public void turnToAngle(double angle) { turnToAngle(0, angle);}

    public void turnToAngle(double fromAngle, double toAngle) {
        double turningDegrees = toAngle - fromAngle; //Get the total defrees to turn
        if (Math.abs(turningDegrees) > 180){
            //Instead of turning all the way around, we turn the opposite direction, this tells how to get those degrees
            //subtract turning degrees from 360 to get the opposite degrees to turn and multiply by minus one to get the turn

            turningDegrees = (360 - turningDegrees) * -1;
        }
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double totalDistanceToMove = getDistanceFor1Degree() * turningDegrees;// get total distance to move in centimeters
        //move the motors
        moveMotors(Speed.turn,totalDistanceToMove,-1 * totalDistanceToMove,MAX_TIMEOUT);

    }

    private double getDistanceFor1Degree(){
        return Math.PI*WHEELS_SPACING_CM / 360;

    }


    public void forward(){
        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
    }

    public void backwards(){
        rightMotor.setDirection(DcMotor.Direction.REVERSE); // Set to Forward if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.FORWARD);// Set to Reverse if using AndyMark motors
    }

    private Speed speed;

    public Speed getSpeed() {
        return speed;
    }

    public void setSpeed(Speed speed) {
        this.speed = speed;
    }

    public void goStraight(Speed speed)
    {
        this.speed = speed;
        leftMotor.setPower(speed.getSpeed());
        rightMotor.setPower(speed.getSpeed());
    }

    public void turn(Turn turn){
        if(Turn.left90.equals(turn) || Turn.right90.equals(turn)){
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double angle_turn = Turn.left90.equals(turn)?-90.0:90.0;     // positive to turn right or left
            // Turn left 90 degree, check wheel diameter and spacing if it's not accurate

            double wheels_turn_cm = 3.14*WHEELS_SPACING_CM * angle_turn/360.0; // wheels distance to turn to the angle
            //opMode.telemetry.addData("Turn distance:", wheels_turn_cm);
            //opMode.telemetry.update();
            moveMotors(Speed.normal, wheels_turn_cm, -wheels_turn_cm, 10);
        }
        else {
            turn(turn.getLeftPower(), turn.getRightPower());
        }
    }

    public void turn(double leftPower, double rightPower){
        if(DcMotor.Direction.REVERSE.equals(rightMotor.getDirection()))
        {
            leftMotor.setPower(rightPower);
            rightMotor.setPower(leftPower);  // turn a litte to get away from the wall
        }
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);  // turn a litte to get away from the wall
    }

    public void go(Speed speed, double distance, double timeout){
        moveMotors(speed, distance, distance, timeout);
    }

    public void go( Speed speed, double distance){
        MAX_TIMEOUT = 30;
        moveMotors(speed, distance, distance, 5);
    }


    private void moveMotors2(Speed speed, double leftDistance, double rightDistance, double timeout) {
        int newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftDistance * COUNTS_PER_CM);
        int newRightTarget = rightMotor.getCurrentPosition() + (int)(rightDistance * COUNTS_PER_CM);
        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.
        runtime.reset();
        leftMotor.setPower(Math.abs(speed.getSpeed()));
        rightMotor.setPower(Math.abs(speed.getSpeed()));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while ((runtime.seconds() < timeout) &&
                (isMoving())) {


        }

        // Stop all motion;
        this.stop();

        // Turn off RUN_TO_POSITION
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isMoving(){
        return leftMotor.isBusy() || rightMotor.isBusy();


    }

    public void stop(){
        leftMotor.setPower(0.0);    // stop motors
        rightMotor.setPower(0.0);
    }
    private void moveMotors(Speed speed, double leftDistance, double rightDistance, double timeout) {
        int newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftDistance * COUNTS_PER_CM);
        int newRightTarget = rightMotor.getCurrentPosition() + (int)(rightDistance * COUNTS_PER_CM);
        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        //setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reset the timeout time and start motion.
        leftMotor.setPower(Math.signum(leftDistance)*Math.abs(speed.getSpeed()));
        rightMotor.setPower(Math.signum(rightDistance)*Math.abs(speed.getSpeed()));
        runtime.reset();

        // keep looping while we are still active, and there is time left, and both motors are running.
       while ( runtime.seconds() < 10 && ( checkDistance(leftDistance,newLeftTarget, leftMotor) || checkDistance(rightDistance,newRightTarget, rightMotor))) {

           /*
           try {
               ((LinearOpMode)opMode).waitOneFullHardwareCycle();
           } catch (InterruptedException e) {
               //do nothing
           }
           */
       }

        // Stop all motion;
        this.stop();

        // Turn off RUN_TO_POSITION
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private boolean checkDistance(double distance, int target, DcMotor motor) {

        if (distance >0) {
            if (motor.getCurrentPosition() >= target) {
                motor.setPower(0.0);
            }
            return motor.getCurrentPosition() < target;
        }

        if (motor.getCurrentPosition() <= target) {
            motor.setPower(0.0);
        }
        return motor.getCurrentPosition() > target;
    }
}
