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
    private int moveDirection = 1;

    /**
     * Main method of {@code this}.
     * Initializes {@code this}.
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

        /* Calculate the absolute bearing of the target. */
        final double absoluteBearing = Math.toRadians( ( getHeading() + e.getBearing() ) % 360 );

        /* Calculate the transform of the target. */
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

        /* Calculate the shortest arc between the gun heading and the target */
        final double shortestArc = Utility.signedAngleDifference( getGunHeadingRadians(), target.getTheta() );

        /* Turn the gun with this difference. */
        setTurnGunRightRadians( shortestArc );
    }

    /**
     * Locks gun on enemy current position.
     * Naive method.
     * @param e a scanned robot.
     */
    private void lockOn( ScannedRobotEvent e ) {

        /* Lock-on radar */
        setTurnRadarLeftRadians( getRadarTurnRemainingRadians() );

        Vector2 predictedPosition = intercept(
                targetTransform.getPosition() ,
                targetTransform.getTrajectory().multiply(targetTransform.getVelocity())
        );

        /* Restrain predictedPosition such that it is bounded by the battlefield. */
        Vector2 restrainedPredictedAbsolutePosition = new Vector2(
                Utility.limit( predictedPosition.getX() + getX(), getWidth(), getBattleFieldWidth()  - getWidth()  ),
                Utility.limit( predictedPosition.getY() + getY(), getHeight(),getBattleFieldHeight() - getHeight() )
        );

        /* Reset the reference frame to this. */
        Vector2 restrainedPredictedRelativePosition = restrainedPredictedAbsolutePosition.subtract( getPosition() );
        
        /* Take aim */
        aimGun( restrainedPredictedRelativePosition );

        if( getGunTurnRemaining() < accuracy && getGunHeat() == 0) setFire(3);

        //TODO: Make movement logic. This movement is dummy behaviour.
        setAhead(6000 * moveDirection);
    }
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
     * This method takes origo as its reference point, in order to generalize this method
     * to use other reference points, transform all the relevant vectors such that the reference
     * point becomes {@code (0,0)}.
     * Uses convergence to estimate the future position of {@code coordinates} to a degree of accuracy.
     * @param coordinates the initial coordinates of a target.
     * @param trajectory the trajectory of a target.
     * @return the coordinate to intercept.
     */
    private Vector2 intercept( Vector2 coordinates, Vector2 trajectory ){

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

    /**
     * Gets the coordinates of {@code this} robot as a {@code Vector2}.
     * @return the coordinates of {@code this}.
     */
    private Vector2 getPosition(){
        return new Vector2( getX(), getY() );
    }
}
