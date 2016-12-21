package org.firstinspires.ftc.teamcode.core;

/**
 * Created by Gary on 12/20/16.
 */

public enum Picture {
    gears(-(12 * 12 - 2) * 25.4f / 2.0f, -12 * 25.4f) ,
    tools(-(12 * 12 - 2) * 25.4f / 2.0f,36 * 25.4f) ,
    legos(-36 * 25.4f, (12 * 12 - 2) * 25.4f / 2.0f) ,
    wheels(12 * 25.4f, (12 * 12 - 2) * 25.4f / 2.0f);
    Picture(double x, double y){
        this.x = x;
        this.y = y;
    }
    private double x;
    private double y;

    public double getX(){
        return this.x;
    }

    public double getY(){
        return this.y;
    }
}
