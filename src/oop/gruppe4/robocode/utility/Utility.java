package oop.gruppe4.robocode.utility;

/**
 * Utility functions not directly tied to robots, but heavily used by robots.
 * @author Ask Hetland Berentsen
 */
public final class Utility {

    /**
     * Calculates the signed smallest angle difference between two angles.
     * This method calculates the relative angle between two angles, using {@code alpha} as the frame of reference.
     * @param alpha the angle to base the calculation on
     * @param beta the angle to calculate the radial distance of.
     * @return A signed radian between {@code 0} and {@code PI} if the shortest radial path between
     *         {@code alpha} and {@code beta} is in the clockwise direction, or a signed radian between
     *         {@code -0} and {@code -PI} if the shortest radial path is in the counterclockwise direction.
     */
    public static double signedAngleDifference( double alpha, double beta ){

        /* Find the normal angles between alpha and beta,
           both in the clockwise direction and in the counterclockwise direction */
        double clockwiseDifference        = robocode.util.Utils.normalAbsoluteAngle( beta  - alpha );
        double counterClockwiseDifference = robocode.util.Utils.normalAbsoluteAngle( alpha - beta  );

        /* Return the (signed) smallest difference between alpha and beta */
        return  clockwiseDifference < counterClockwiseDifference ? clockwiseDifference : -counterClockwiseDifference;
    }

    /**
     * Limits a number between a lower and upper bound.
     * @param value the value to limit.
     * @param min the lower bound.
     * @param max the upper bound
     * @return the constrained value.
     */
    public static double limit( double value, double min, double max ){
        return Math.max( min, Math.min(value, max) );
    }
}
