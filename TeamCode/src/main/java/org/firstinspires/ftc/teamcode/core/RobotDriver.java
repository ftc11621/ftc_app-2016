package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RobotDriver {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    //private double timeoutS = 30;

    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_CM       = 9.15 ;     // For figuring circumference
    private static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * Math.PI );
    private LinearOpMode opMode = null;

    private static final double     WHEELS_SPACING_CM       = 34.3;     // spacing between wheels for turns

    private ElapsedTime runtime = new ElapsedTime();
    private int MAX_TIMEOUT = 30;

    public RobotDriver(LinearOpMode opMode, HardwareMap hardwareMap ) {
        this.leftMotor  = hardwareMap.dcMotor.get("motor_2");
        this.rightMotor = hardwareMap.dcMotor.get("motor_1");
        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        this.opMode = opMode;
    }

    public void setRunMode(DcMotor.RunMode runMode){
        leftMotor.setMode(runMode);
        rightMotor.setMode(runMode);
    }

    public void turnToAngle(double fromAngle, double toAngle) {
        //todo implement
    }

    public static enum Direction { forward, back}
    public static enum Turn {slightLeft(0.05,0.15), hardLeft(0,0.1), slightRight(0.15,0.05), hardRight(0.1,0), left90(0,0.5), right90(0.5,0);
        double leftPower = 0;
        double rightPower = 0;

        Turn(double leftPower, double rightPower) {
            this.leftPower = leftPower;
            this.rightPower = rightPower;
        }

        public double getLeftPower() {
            return leftPower;
        }

        public double getRightPower() {
            return rightPower;
        }
    }

    private Speed speed;

    public static enum Speed {fast(1.0),normal(0.5), slow(0.25), turn(0.2);

        Speed(double speed) {
          this.speed = speed;
        }
        double speed;
        public double getSpeed() {return this.speed;}
    }

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
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);  // turn a litte to get away from the wall
    }

    public void go(Speed speed, double distance, double timeout){
        moveMotors(speed, distance, distance, timeout);
    }

    public void go( Speed speed, double distance){
        MAX_TIMEOUT = 30;
        moveMotors(speed, distance, distance, MAX_TIMEOUT);
    }


    private void moveMotors(Speed speed, double leftDistance, double rightDistance, double timeout) {
        int newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftDistance * COUNTS_PER_CM);
        int newRightTarget = rightMotor.getCurrentPosition() + (int)(rightDistance * COUNTS_PER_CM);
        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftMotor.setPower(Math.abs(speed.getSpeed()));
        rightMotor.setPower(Math.abs(speed.getSpeed()));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (isMoving())) {

            // Display it for the driver.
            //opMode.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            //opMode.telemetry.addData("Path2",  "Running at %7d :%7d",
            //        leftMotor.getCurrentPosition(),
            //        rightMotor.getCurrentPosition());
            //opMode.telemetry.update();
        }

        // Stop all motion;
        this.stop();

        // Turn off RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isMoving(){
        return leftMotor.isBusy() && rightMotor.isBusy();
    }

    public void stop(){
        leftMotor.setPower(0.0);    // stop motors
        rightMotor.setPower(0.0);
    }


}
