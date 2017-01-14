package org.firstinspires.ftc.teamcode.core;

/**
 * Created by Gary on 12/20/16.
 */

public enum Picture {
    gears(-71, -12 ) ,
    tools(-71,36 ) ,
    legos(-36 , 71) ,
    wheels(12 , 71);
    Picture(int x, int y){
        this.x = x ;
        this.y = y;
    }
    private double x;
    private double y;

    public double getX(){
        return this.x*25.4f;
    }

    public double getY(){
        return this.y*25.4f;
    }
    public double getX(int inches){
        if (this.x == -71) {
            return (this.x + inches)*25.4;
        }
        return getX();

    }
    public double getY(int inches){
        if (this.y == 71) {
            return (this.y - inches)*25.4;
        }
        return getY();

    }
}
