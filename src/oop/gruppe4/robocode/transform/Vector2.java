package oop.gruppe4.robocode.transform;

import static java.lang.Math.*;

public class Vector2 {
    private double x, y;
    public Vector2( double x, double y ){
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }
    public double getY(){
        return y;
    }
    public Vector2 add( Vector2 v ){
        return new Vector2( this.x + v.x, this.y + v.y );
    }
    public Vector2 multiply( double scalar ){
        return new Vector2( this.x * scalar, this.y * scalar );
    }
    public double getScalar(){
        return sqrt(x*x + y*y);
    }
    public double getTheta(){
        return (Math.atan2(x,y) + 2*PI) % (2*PI);
    }
    public Vector2 subtract( Vector2 v ){
        return new Vector2(this.x - v.x, this.y - v.y);
    }
    public double distance( Vector2 v ){
        return this.subtract( v ).getScalar();
    }
}
