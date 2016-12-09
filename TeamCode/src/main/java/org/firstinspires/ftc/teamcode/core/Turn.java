package org.firstinspires.ftc.teamcode.core;

/**
 * Created by Marie on 12/8/2016.
 */
public enum Turn {
    slightLeft(0.05, 0.15), hardLeft(0, 0.1), slightRight(0.15, 0.05), hardRight(0.1, 0), left90(0, 0.5), right90(0.5, 0);
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
