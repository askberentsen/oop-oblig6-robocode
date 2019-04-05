package oop.gruppe4.robocode.robot;

import oop.gruppe4.robocode.transform.Transform;
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

    private Transform targetTransform;
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
        final double absoluteBearing = Math.toRadians( ( getHeading() + e.getBearing() ) % 360 );

        targetTransform = new Transform(
                ( Math.sin( absoluteBearing ) * e.getDistance() ),
                ( Math.cos( absoluteBearing ) * e.getDistance() ),
                ( Math.sin( e.getHeadingRadians() ) ),
                ( Math.cos( e.getHeadingRadians() ) ),
                e.getVelocity()
        );
    }

    /**
     * Aims the gun at a relative coordinate.
     * @param target the target coordinate.
     */
    private void aimGun( Vector2 target ){

        final double shortestArc = Utility.signedAngleDifference( getGunHeadingRadians(), target.getTheta() );

        setTurnGunRightRadians( shortestArc );
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

        Vector2 predictedPosition = intercept(
                targetTransform.getPosition() ,
                targetTransform.getTrajectory().multiply(targetTransform.getVelocity())
        );

        /* Take aim */
        aimGun( predictedPosition );

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

    private Vector2 getPosition(){
        return new Vector2( getX(), getY() );
    }
}
