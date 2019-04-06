package oop.gruppe4.robocode.transform;

import org.jetbrains.annotations.NotNull;

import static java.lang.Math.*;

/**
 * A two-dimensional vector using {@code double} coordinates.
 * {@code Vector2} is an immutable class.
 * @author Ask Hetland Berentsen
 */
public class Vector2 {

    /**
     * The x coordinate.
     */
    final private double x;

    /**
     * The y coordinate.
     */
    final private double y;

    /**
     * Class constructor.
     * @param x the x coordinate.
     * @param y the y coordinate.
     */
    public Vector2( double x, double y ){
        this.x = x;
        this.y = y;
    }

    /**
     * Gets the x coordinate of {@code this}.
     * @return a {@code double}
     */
    public double getX() {
        return x;
    }

    /**
     * Gets the y coordinate of {@code this}.
     * @return a {@code double}
     */
    public double getY(){
        return y;
    }

    /**
     * Adds two vectors together.
     * @param v another {@code Vector2}.
     * @return a {@code new Vector2}.
     */
    public Vector2 add( @NotNull Vector2 v ){
        return new Vector2( this.x + v.x, this.y + v.y );
    }

    /**
     * Multiplies {@code this} with the scalar {@code scalar}.
     * @param scalar a scalar to multiply with.
     * @return a {@code new Vector2}.
     */
    public Vector2 multiply( double scalar ){
        return new Vector2( this.x * scalar, this.y * scalar );
    }

    /**
     * Calculates the scalar of {@code this}.
     * @return a {@code double}.
     */
    public double getScalar(){
        return sqrt(x*x + y*y);
    }

    /**
     * Calculates the angle of {@code this}.
     * @return an unsigned radian between {@code 0} and {@code 2 PI} (Not including {@code 2 PI})
     *          where {@code 0} is north, {@code PI/2} is east, {@code PI} is south and {@code 3/2 PI} is west.
     */
    public double getTheta(){
        return (atan2(x,y) + 2*PI) % (2*PI);
    }

    /**
     * Rotates {@code this} to a new {@code Vector2} using the angle {@code theta}.
     * @param theta the radians to rotate {@code this} with. A {@code double} between
     * {@code 0} and {@code 2 PI} (Not including {@code 2 PI}).
     * @return a {@code new Vector2}.
     */
    public Vector2 rotate( double theta ){
        double scalar = getScalar();
        return new Vector2( sin(theta)*scalar, cos(theta)*scalar );
    }

    /**
     * Subtracts a {@code Vector2} from {@code this}.
     * @param v another {@code Vector2}
     * @return a {@code new Vector2}.
     */
    public Vector2 subtract( @NotNull Vector2 v ){
        return new Vector2(this.x - v.x, this.y - v.y);
    }

    /**
     * Gets the distance between two vectors.
     * @param v another {@code Vector2}.
     * @return a {@code new Vector2}.
     */
    public double distance( @NotNull Vector2 v ){
        return this.subtract( v ).getScalar();
    }

    @Override
    public String toString(){
        return String.format("<%04f,%04f>", x, y);
    }
}
