package org.firstinspires.ftc.teamcode.core;

/**
 * Created by Marie on 12/8/2016.
 */
public enum Speed {
    fast(1.0), normal(0.5), slow(0.25), turn(0.2), speed1(0.15), speed2(0.2), speed3(0.3),
    speed4(0.4), speed5(0.5), speed6(0.6), speed7(0.7), speed8(0.8), speed9(0.9), speed10(1.0);

    Speed(double speed) {
        this.speed = speed;
    }

    double speed;

    public double getSpeed() {
        return this.speed;
    }
}
