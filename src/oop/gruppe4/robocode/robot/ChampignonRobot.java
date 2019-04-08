package oop.gruppe4.robocode.robot;

import oop.gruppe4.robocode.transform.Transform;
import oop.gruppe4.robocode.transform.Vector2;
import oop.gruppe4.robocode.utility.Utility;
import robocode.AdvancedRobot;
import robocode.HitWallEvent;
import robocode.ScannedRobotEvent;

import java.util.HashMap;
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

    private HashMap<String, RobotStatistics> history = new HashMap<>();
    private String target;
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

        target = e.getName();
        if( !history.containsKey(target) ){
            history.put(target, new RobotStatistics(30));
        }

        /* Calculate the absolute bearing of the target. */
        final double absoluteBearing = Math.toRadians( ( getHeading() + e.getBearing() ) % 360 );

        /* Calculate the transform of the target. */
        targetTransform = new Transform(
                ( Math.sin( absoluteBearing ) * e.getDistance() ) + getX(),
                ( Math.cos( absoluteBearing ) * e.getDistance() ) + getY(),
                ( Math.sin( e.getHeadingRadians() ) ),
                ( Math.cos( e.getHeadingRadians() ) ),
                e.getVelocity()
        );
        history.get(target).add(targetTransform);
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
                targetTransform.getPosition().subtract(getPosition()) ,
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
     * Calculates a coordinate to intercept if a target is moving in a circle.
     * @param referenceFrame the coordinate to use as a reference frame.
     * @param coordinates the coordinate of the target.
     * @param trajectory the direction and velocity of the target.
     * @param oldTrajectory the direction and velocity the target when the target was last calculated.
     * @param deltaTime the amount of ticks since {@code oldTrajectory} was calculated.
     * @return a coordinate to intercept.
     */
    private Vector2 circularIntercept( Vector2 referenceFrame, Vector2 coordinates, Vector2 trajectory, Vector2 oldTrajectory, long deltaTime ){

        /* The difference in angle. */
        double angleDifference = Utility.signedAngleDifference( oldTrajectory.getTheta(), trajectory.getTheta() );

        /* The angle of one step */
        final double THETA = angleDifference / deltaTime;

        /* The length of the arc after one step. */
        final double ARCLENGTH = trajectory.getScalar();

        /* The radius of the pivot. */
        final double PIVOT_RADIUS = Math.abs(ARCLENGTH / THETA);

        /* The handedness of the perpendicular vector. */
        int handedness = THETA >= 0 ? -1 : 1;

        /* The vector perpendicular to the trajectory. */
        Vector2 perpendicularVector = new Vector2( trajectory.getY()*handedness, trajectory.getX() * (-handedness) );

        /* The vector to pivot from. */
        final Vector2 PIVOT = perpendicularVector.multiply( PIVOT_RADIUS / perpendicularVector.getScalar() );

        //TODO: make bulletVelocity dynamic. As of now its hard-coded.
        double bulletVelocity = 11;

        /* The predicted position. */
        Vector2 nextPosition = coordinates;

        /* The amount of ticks until the intercept will happen. */
        double steps = 1;
        double previousSteps = 0;

        /* Continue to predict until the difference in predictions is sufficiently low. */
        while( Math.abs(steps-previousSteps) > 0.2 ) {
            previousSteps = steps;

            /* The amount of ticks until the intercept can be no lower than the amount of ticks
               until the bullet hits the current predicted position. */
            steps = nextPosition.distance( referenceFrame ) / bulletVelocity;

            /* Calculate the arc as the pivot rotated by the amount of steps, then subtract the original pivot. */
            Vector2 arc = PIVOT.rotate(-THETA * steps).subtract( PIVOT );

            /* Predict the position as the current position plus an arc. */
            nextPosition = coordinates.add( arc );

            /* If the predicted position is not inbounds, break. */
            if( !nextPosition.isContained(15, 15, getBattleFieldWidth()-15, getBattleFieldHeight() - 15) )
                break;
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
