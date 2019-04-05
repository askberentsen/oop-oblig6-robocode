package oop.gruppe4.robocode.robot;

import oblig6.vector.Vector;
import oop.gruppe4.robocode.transform.Vector2;
import oop.gruppe4.robocode.utility.Utility;
import robocode.AdvancedRobot;
import robocode.HitWallEvent;
import robocode.ScannedRobotEvent;

/*
 * TODO: 02/04/2019 | ALL
 * TODO: Implement
 * TODO: Add Yeet function
 */

/**
 * The main robot of the project.
 * @author Gruppe 4.
 */
public class ChampignonRobot extends AdvancedRobot {

    private int targetX, targetY;
    private double targetDX, targetDY;

    private double leadingFactor = 23;
    private int accuracy = 4;

    /**
     * Main method of this {@code ChampignonRobot}.
     */
    @Override
    public void run() {
        setAdjustRadarForRobotTurn( true );
        setAdjustGunForRobotTurn  ( true );
        setAdjustRadarForGunTurn  ( true );
        turnRadarRightRadians( Double.POSITIVE_INFINITY );
    }

    /**
     * Calculates the coordinates and movement of an enemy robot.
     * @param e the robot that was scanned.
     */
    private void targetRobot( ScannedRobotEvent e ) {
        //TODO: Calculate the values of the target

        /* Calculate the absolute bearing of the target. */
        double absoluteBearing = Math.toRadians( ( getHeading() + e.getBearing() ) % 360 );

        /* Calculate the coordinates. */
        targetX = (int)( Math.sin( absoluteBearing ) * e.getDistance() );
        targetY = (int)( Math.cos( absoluteBearing ) * e.getDistance() );

        /* Calculate the absolute angle the enemy is moving towards. */
        double angle = e.getHeadingRadians();

        /* Calculate the relative later coordinates */
        targetDX = ( Math.sin( angle ) * e.getVelocity() );
        targetDY = ( Math.cos( angle ) * e.getVelocity() );
    }

    /**
     * Aims the gun at a relative coordinate.
     * @param x the relative x coordinate of the target point.
     * @param y the relative y coordinate of the target point.
     */
    private void aimGun( double x, double y ){
        double target = (Math.atan2( x, y ) + ( 2*Math.PI )) % ( 2*Math.PI );

        double current = getGunHeadingRadians();
        double diff = Utility.signedAngleDifference( current, target );

        setTurnGunRightRadians(diff);
    }

    /**
     * Locks gun on enemy current position.
     * Naive method.
     * TODO: Use absolute data instead of relative data. Base aiming on coordinates (#targetRobot()).
     * @param e a scanned robot.
     */
    private void lockOn( ScannedRobotEvent e ){

        /* Lock-on radar */
        setTurnRadarLeftRadians( getRadarTurnRemainingRadians() );

        Vector2 predictedPosition = intercept( new Vector2(targetX, targetY) , new Vector2(targetDX, targetDY));

        /* Take aim */
        aimGun( predictedPosition.getX(), predictedPosition.getY() );

        if( getGunTurnRemaining() < accuracy && getGunHeat() == 0) setFire(3);

        //TODO: Make movement logic. This movement is dummy behaviour.
        setAhead(6000 * moveDirection);
    }
    private int moveDirection = 1;
    public void onHitWall(HitWallEvent e){
        moveDirection=-moveDirection;//reverse direction upon hitting a wall
    }

    @Override
    public void onScannedRobot( ScannedRobotEvent e ) {
        targetRobot(e); //targets the robot coordinates and vector of enemy
        lockOn(e);
    }

    /**
     * Finds the intercept coordinates of a target.
     * Uses lookahead to converge on a position where the bullet will hit the target.
     * @param coordinates the initial coordinates of a target.
     * @param trajectory the trajectory of a target.
     * @return the coordinate to intercept.
     */
    private Vector2 intercept(Vector2 coordinates, Vector2 trajectory){

        /* The velocity of a bullet */
        final double bulletVelocity = 11;

        /* The amount of steps before the intercept. */
        double steps = 1;
        double previousSteps = 0;

        /* The lookahead position of the target. */
        Vector2 nextPosition = coordinates;

        /* Iterate until the difference between the steps is less than 0.5 */
        while( Math.abs(steps-previousSteps) > 0.2 ){
            previousSteps = steps;
            /* Set the amount of steps to check to the distance to the next
               position divided by the bullet velocity */
            steps = nextPosition.getScalar() / bulletVelocity;

            /* Set the next position to coordinates + trajectory*steps. */
            nextPosition = coordinates.add(  trajectory.multiply(steps) );
        }

        return nextPosition;
    }
}
