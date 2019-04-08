package oop.gruppe4.robocode.robot;

import oop.gruppe4.robocode.transform.Transform;
import oop.gruppe4.robocode.transform.Vector2;
import oop.gruppe4.robocode.utility.Utility;
import robocode.*;
import robocode.util.Utils;

import java.util.ArrayList;
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

    private RadarStatus status = RadarStatus.SCANNING;

    private HashMap<String, RobotStatistics> history = new HashMap<>();
    private ArrayList<Transform> virtualBullets = new ArrayList<>();
    private String targetName;
    private double wallHitCooldown = 0;
    private int moveDirection = 1;
    private double velocity = 8;

    private double analyzingDuration = 0;

    /**
     * Main method of {@code this}.
     * Initializes {@code this}.
     */
    @Override
    public void run() {
        setAdjustRadarForRobotTurn( true );
        setAdjustGunForRobotTurn  ( true );
        setAdjustRadarForGunTurn  ( true );
        beginScan();
    }

    /**
     * Update on every tick.
     * @param e the status event.
     */
    @Override
    public void onStatus(StatusEvent e) {
        System.out.println(status.name());
        for( Transform virtualBullet : virtualBullets ){
            virtualBullet.update();
            if( !virtualBullet.getPosition().isContained( 0, 0,getBattleFieldWidth(), getBattleFieldHeight() )){
                virtualBullets.remove( virtualBullet );
            }
        }
    }

    /**
     * Logs the statistics of a target.
     * @param e the target {@code ScannedRobotEvent}.
     */
    private void logTarget( ScannedRobotEvent e ) {

        final String ROBOT_NAME = e.getName();

        /* target if e is the first scan. */
        if( targetName == null ) {
            targetName = ROBOT_NAME;
            beginAnalyze();
        }
        if( !history.containsKey( ROBOT_NAME ) ) history.put( ROBOT_NAME, new RobotStatistics(30) );

        /* Calculate the absolute bearing of the target. */
        final double ABSOLUTE_BEARING = Math.toRadians( ( getHeading() + e.getBearing() ) % 360 );
        final double ROBOT_X  = ( Math.sin( ABSOLUTE_BEARING ) * e.getDistance() ) + getX();
        final double ROBOT_Y  = ( Math.cos( ABSOLUTE_BEARING ) * e.getDistance() ) + getY();
        final double ROBOT_DX = ( Math.sin( e.getHeadingRadians() ) );
        final double ROBOT_DY = ( Math.cos( e.getHeadingRadians() ) );
        final double ROBOT_VELOCITY = e.getVelocity();
        final long   TIMESTAMP = e.getTime();

        final double ENERGY = e.getEnergy();

        /* Calculate the statistics of the target. */
        RobotStatistics.Statistic targetStatistic = new RobotStatistics.Statistic(
                ROBOT_X,
                ROBOT_Y,
                ROBOT_DX,
                ROBOT_DY,
                ROBOT_VELOCITY,
                TIMESTAMP
        );
        RobotStatistics statistics = history.get( ROBOT_NAME );
        statistics.add(targetStatistic);
        statistics.setEnergy( ENERGY );
    }

    public void onHitWall(HitWallEvent e){
    private void lockScanner() {
        setTurnRadarLeftRadians( getRadarTurnRemainingRadians() );
    }

    private void aimGun() {

        final RobotStatistics TARGET_STATISTICS = history.get(targetName);
        final RobotStatistics.Statistic CURRENT_TARGET_STAT = TARGET_STATISTICS.getLast();
        final RobotStatistics.Statistic PREVIOUS_TARGET_STAT = TARGET_STATISTICS.getPrevious();

        Vector2 targetCoordinates = CURRENT_TARGET_STAT.getPosition();
        Vector2 targetTrajectory = CURRENT_TARGET_STAT.getTrajectory();
        Vector2 targetOldTrajectory = PREVIOUS_TARGET_STAT.getTrajectory();
        long deltaTime = CURRENT_TARGET_STAT.getTimeStamp() - PREVIOUS_TARGET_STAT.getTimeStamp();

        final double angularDifference = Utility.signedAngleDifference(
                targetOldTrajectory.getTheta(),
                targetTrajectory.getTheta()
        );

        Vector2 predictedPosition;

        /* Direct interception */
        if( targetTrajectory.getScalar() == 0.0 || targetTrajectory.subtract( targetOldTrajectory ).getScalar() < 0.5){
            predictedPosition = targetCoordinates;
        }
        /* Linear interception. */
        else if( Math.abs(angularDifference) < 0.05 ){
            predictedPosition = linearIntercept( getPosition(), targetCoordinates, targetTrajectory );
        }
        /* Circular interception. */
        else{
            predictedPosition = circularIntercept( getPosition(), targetCoordinates, targetTrajectory, targetOldTrajectory, deltaTime );
        }

        double theta = Utility.signedAngleDifference( getGunHeadingRadians(), predictedPosition.subtract(getPosition()).getTheta() );
        setTurnGunRightRadians( theta );
    }

        if( wallHitCooldown <= 0 ){
            moveDirection *= -1;
            wallHitCooldown = 2;
        }
        else{
            wallHitCooldown--;
        }
    }

    @Override
    public void onScannedRobot( ScannedRobotEvent e ) {

        logTarget(e);

        final RobotStatistics.Statistic CURRENT_TARGET_STAT = history.get(targetName).getLast();

        Vector2 lastPosition = CURRENT_TARGET_STAT.getPosition();
        Vector2 relativePosition = lastPosition.subtract( this.getPosition() );

        //TODO: Does not account for if the target is no longer on the battlefield (dead).
        switch ( status ){

            /* The robot is scanning.
             * Continue to scan until the robot has scanned at least 360 degrees,
             * or if the robot has scanned the target again after having scanned 180 degrees.
             */
            case SCANNING:
                System.out.println("Scanning... found " + e.getName() );
                /* If the robot has completed the 360 degree scan, begin targeting */
                if( getRadarTurnRemainingRadians() < 3 * Math.PI && e.getName().equals(targetName) ){
                    beginAnalyze();
                }
                else if( getRadarTurnRemainingRadians() < 2 * Math.PI ){ //Finished scanning
                    beginTarget();
                }
                break;

             /* The robot is targeting.
              * Turn radar to the targets last known position. If the target wasn't found,
              * continue to turn until the target is found.
              */
            case TARGETING:
                System.out.println( String.format("Targeting %s @ %s", targetName, lastPosition) );

                if( e.getName().equals(targetName) ){
                    beginAnalyze();
                }
                else {
                    /* Aim radar on target */
                    double theta = Utility.signedAngleDifference( getRadarHeadingRadians(), relativePosition.getTheta() );


                    /* If target moved, add 1 to turn */
                    double secureTheta = theta >= 0.0 ? theta + 1 : theta - 1;

                    setTurnRadarRightRadians( secureTheta );
                }
                break;

            /* The robot is analyzing the target.
             * Analyze the target at least 2 times to gather enough information to predict
             * the targets future position to a reasonable degree of accuracy.
             */
            case ANALYZING:

                System.out.println( "Analyzing " + e.getName() + "..." );
                analyzingDuration++;

                /* Lock radar on target. */
                lockScanner();

                aimGun();

                if( analyzingDuration > 2 ){
                    beginEngage();
                }
                break;

            /* The robot is actively engaging the target.
             * Continue to predict the targets position and aim until the gun has cooled down.
             * When the gun is cool, shoot and start scanning again.
             */
            case ENGAGING:
                System.out.println( String.format("Engaging %s @ %s", e.getName(), lastPosition) );
                lockScanner();
                aimGun();

                if( getGunHeat() == 0 && getGunTurnRemaining() < 1 ) {
                    System.out.println( String.format("Shooting %s @ %s", e.getName(), lastPosition) );
                    setFire(3);
                    beginScan();
                }
                break;

        }
    }

    @Override
    public void onHitByBullet( HitByBulletEvent e ) {
        targetName = e.getName();

    }

    /**
     * Initializes the {@code SCANNING} status.
     */
    private void beginScan(){
        status = RadarStatus.SCANNING;
        /* Scan 720 degrees in case target has moved. */
        setTurnRadarRightRadians( 4 * Math.PI );
    }

    /**
     * Initializes the {@code ENGAGING} status.
     */
    private void beginEngage(){
        status = RadarStatus.ENGAGING;
    }

    /**
     * Initializes the {@code ANALYZING} status.
     */
    private void beginAnalyze(){
        analyzingDuration = 0;
        setTurnRadarRightRadians( Double.NEGATIVE_INFINITY );
        status = RadarStatus.ANALYZING;
    }
    /**
     * Initializes the {@code TARGETING} status.
     */
    private void beginTarget(){
        status = RadarStatus.TARGETING;
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
    private Vector2 linearIntercept( Vector2 referenceFrame, Vector2 coordinates, Vector2 trajectory ) {

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
            steps = nextPosition.distance(referenceFrame) / bulletVelocity;

            /* Set the next position to coordinates + trajectory*steps. */
            nextPosition = coordinates.add(  trajectory.multiply(steps) );
            if( !nextPosition.isContained( 15, 15, getBattleFieldWidth()-15, getBattleFieldHeight() - 15 ) )
                break;
        }

        return nextPosition;
    }

    /**
     * Calculates a coordinate to intercept if a target is moving in a circle.
     * To find the intercept coordinate, find the angular difference between two trajectories and account for the time difference.
     * Use this angle to calculate the radius of a pivot vector. The angle of the vector should be perpendicular to the
     * trajectory. When the pivot vector is rotated by the angle that was calculated, the arc-length is equal to
     * the scalar of the {@code trajectory} vector.
     * Uses convergence to estimate the future position of {@code coordinates} to a degree of accuracy.
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
        double bulletVelocity = 11; // 20 - ( 3 * bulletpower )

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
            Vector2 arc = PIVOT.rotate( -THETA * steps).subtract( PIVOT );

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

    private enum RadarStatus {

        /**
         * The robot is scanning enemies.
         * Scan 360 degrees before moving on.
         */
        SCANNING,

        /**
         * Trying to find a specific target.
         * Finished scanning enemies, moving scanner to a specific target.
         */
        TARGETING,

        /**
         * Analyzing a locked-on target.
         * Analyze for at least two ticks to get fresh data and deltas.
         */
        ANALYZING,

        /**
         * Actively engaging a locked-on target.
         * Aim and shoot.
         */
        ENGAGING
    }
}
