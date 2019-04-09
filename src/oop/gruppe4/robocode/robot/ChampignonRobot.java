package oop.gruppe4.robocode.robot;

import oop.gruppe4.robocode.transform.Transform;
import oop.gruppe4.robocode.transform.Vector2;
import oop.gruppe4.robocode.utility.Utility;
import org.jetbrains.annotations.NotNull;
import robocode.*;

import java.util.ArrayList;
import java.util.HashMap;


// TODO: 08/04/2019 does not yet account for enemies killing other enemies. Call disengage(String) when the enemy cannot be found.

/**
 * The main robot of the project.
 * @author Gruppe 4.
 */
public class ChampignonRobot extends AdvancedRobot {

    /**
     * The state of the scanner.
     * @see RadarStatus
     */
    private RadarStatus status = RadarStatus.SCANNING;

    /**
     * The history of the targets.
     */
    private final HashMap<String, RobotStatistics> history = new HashMap<>();

    /**
     * A list of virtual bullets.
     * <p>
     *     To avoid enemies the robot keeps track of virtual bullets the enemies could shoot, and
     *     tries to avoid them.
     * </p>
     * @see #onStatus(StatusEvent)
     */
    private ArrayList<Transform> virtualBullets = new ArrayList<>();

    /**
     * The name of the enemy to target.
     */
    private String targetName;

    /**
     * Main method of {@code this}.
     */
    @Override
    public void run() {
        /* We want to know whether a robot has been scanned or not for every tick.
         * ScannedRobotEvent is set to a higher priority than StatusEvent, so that
         * we can check if a target has been scanned. */
        super.setEventPriority("ScannedRobotEvent", 99);
        super.setEventPriority("StatusEvent", 98);

        setAdjustRadarForRobotTurn( true );
        setAdjustGunForRobotTurn  ( true );
        setAdjustRadarForGunTurn  ( true );
        beginScan();
    }

    /**
     * Logs the statistics of a target.
     * @param e the target {@code ScannedRobotEvent}.
     */
    private void logTarget( @NotNull ScannedRobotEvent e ) {

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
                TIMESTAMP,
                ENERGY
        );

        RobotStatistics statistics = history.get( ROBOT_NAME );
        statistics.add( targetStatistic );
    }

    /**
     * Locks the scanner.
     * Flips the rotation of the scanner.
     */
    private void lockScanner() {
        setTurnRadarLeftRadians( getRadarTurnRemainingRadians() );
    }

    /**
     * Aims the gun at a target.
     * Estimates a future position and turns the gun to that direction.
     * <p>
     *     If the target is not moving, or not moving a lot on average. The robot will aim towards the
     *     targets current position.
     * </p>
     * <p>
     *     If the target is moving in a straight line, the robot will intercept the target in a straight line.
     *     {@code circularInterception()} is reliant on division by the angle, and when the angle is {@code 0},
     *     the method cannot estimate a coordinate. {@code linearIntercept} is faster than {@code circularIntercept()}
     *     and should be used as often as possible before it.
     * </p>
     * <p>
     *     If the target is moving in a curve, the robot will assume the target will continue to move in the
     *     same curve.
     * </p>
     * @see #linearIntercept(Vector2, Vector2, Vector2)
     * @see #circularIntercept(Vector2, Vector2, Vector2, Vector2, long)
     */
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
        if( targetTrajectory.getScalar() == 0.0 || targetTrajectory.add( targetOldTrajectory ).getScalar() < 0.5){
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

    /**
     * Clears a target from the history and untargets it.
     * @param name the name of a target.
     */
    private void disengage( String name ){
        //history.remove( name );
        /*if( name.equals(targetName) ){
            targetName = null;
        }*/
        beginScan();
    }

    /**
     * Update on every tick.
     * @param e the status event.
     */
    @Override
    public void onStatus( StatusEvent e ) {

        final long TIMESTAMP = e.getTime();

        /* Virtual bullet routine. */
        for( Transform virtualBullet : virtualBullets ){
            virtualBullet.update();
            if( !virtualBullet.getPosition().isContained( 0, 0,getBattleFieldWidth(), getBattleFieldHeight() )){
                virtualBullets.remove( virtualBullet );
            }
        }

        /* History routine. */
        history.forEach( (name, statistics) ->{
            // TODO: 09/04/2019 update targets that were not scanned.
        });

        /* Movement routine. */
        // TODO: 09/04/2019 update self

        /* Scanner routine. */
        switch (status){
            case SCANNING:
                if (getRadarTurnRemainingRadians() < 1.5 * Math.PI) {
                    System.out.println( String.format( "Disengaging %s: could not find target during Scanning phase.", targetName ) );
                    disengage( targetName );
                }
                break;
            case ANALYZING:
                break;
            case ENGAGING:
                break;
        }
    }

    /**
     * This method is called when your robot sees another robot, i.e. when the robot's radar scan "hits" another robot.
     * Uses finite states to decide what actions to perform.
     * <p>
     *     When the robot is scanning, it will scan {@code 180 ~ 720} degrees until it has scanned all enemies or
     *     found the target after the initial {@code 180} degrees.
     *     If the robot finds the target again before the first 360 degrees, it will jump directly to analyze.
           Otherwise, it will continue to turn the radar until it has found another target and will then
     *     switch to targeting.
     * </p>
     * <p>
     *     When the robot is targeting, it will rotate the scanner towards where the target was last seen. If the
     *     target was not found at the expected position, it will continue to turn in the same direction until it does.
     *     When the robot has found the target it will then switch to analyze.
     * </p>
     * <p>
     *     When the robot is analyzing, it will keep the scanner locked on the target for 3 ticks to gather
     *     sufficient data to make a prediction. After the 3 ticks, the robot will switch to engage.
     * </p>
     * <p>
     *     When the robot is engaging, it will continue locking the scanner on the target and wait for the gun to
     *     cool down. the robot will additionally not try to shoot if the target is too far away. After firing a shot,
     *     the robot will initiate a scan while the gun is cooling down.
     * </p>
     * @see #aimGun()
     * @see #beginScan()
     * @see #beginAnalyze()
     * @see #beginEngage()
     * @see RadarStatus
     */
    @Override
    public void onScannedRobot( ScannedRobotEvent e ) {

        /* Log the statistics of the scanned robot */
        logTarget(e);

        /* Get the statistics of the target (or the scanned robot if no robot was targeted) */
        final RobotStatistics STATISTICS = history.get( targetName );

        final RobotStatistics.Statistic CURRENT_TARGET_STAT = STATISTICS.getLast();
        final RobotStatistics.Statistic PREVIOUS_TARGET_STAT = STATISTICS.getPrevious();

        /* Initiate virtualized bullets routine */
        if (CURRENT_TARGET_STAT.getEnergy() - PREVIOUS_TARGET_STAT.getEnergy() < 0) {
            virtualizeBullets(CURRENT_TARGET_STAT);
        }

        Vector2 lastPosition = CURRENT_TARGET_STAT.getPosition();
        Vector2 relativePosition = lastPosition.subtract(this.getPosition());

        //TODO: Does not account for if the target is no longer on the battlefield (dead).
        switch (status) {

            /* The robot is scanning.
             * Continue to scan until the robot has scanned at least 360 degrees,
             * or if the robot has scanned the target again after having scanned 180 degrees.
             */
            case SCANNING:
                /* If the robot has completed the 360 degree scan, begin targeting */
                if (getRadarTurnRemainingRadians() < 3 * Math.PI && e.getName().equals(targetName)) {
                    beginAnalyze();
                }
                break;

            /* The robot is analyzing the target.
             * Analyze the target at least 2 times to gather enough information to predict
             * the targets future position to a reasonable degree of accuracy.
             */
            case ANALYZING:

                lockScanner();

                if (e.getName().equals(targetName)) {
                    aimGun();
                    beginEngage();
                }

                break;

            /* The robot is actively engaging the target.
             * Continue to predict the targets position and aim until the gun has cooled down.
             * When the gun is cool, shoot and start scanning again.
             */
            case ENGAGING:
                lockScanner();

                if (e.getName().equals(targetName)) {
                    aimGun();
                    if (getGunHeat() == 0 && getGunTurnRemaining() < 1 && e.getDistance() < 256 ) {
                        setFire(3);
                        beginScan();
                    }
                }
                break;

        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void onHitByBullet( HitByBulletEvent e ) {
        targetName = e.getName();

    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void onHitWall( HitWallEvent e ) {
        // TODO: 09/04/2019
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void onRobotDeath( RobotDeathEvent e ) {
        disengage( e.getName() );
        history.remove( e.getName() );
    }

    /**
     * Initializes the {@code SCANNING} status.
     * @see RadarStatus#SCANNING
     */
    private void beginScan() {
        status = RadarStatus.SCANNING;
        /* Scan 720 degrees in case target has moved. */
        setTurnRadarRightRadians( 4 * Math.PI );
    }

    /**
     * Initializes the {@code ENGAGING} status.
     * @see RadarStatus#ENGAGING
     */
    private void beginEngage() {
        status = RadarStatus.ENGAGING;
    }

    /**
     * Initializes the {@code ANALYZING} status.
     * @see RadarStatus#ANALYZING
     */
    private void beginAnalyze() {
        setTurnRadarRightRadians( Double.NEGATIVE_INFINITY );
        status = RadarStatus.ANALYZING;
    }

    /**
     * Finds the intercept coordinates of a target.
     * Uses lookahead to converge on a position where the bullet will hit the target.
     * This method takes origo as its reference point, in order to generalize this method
     * to use other reference points, transform all the relevant vectors such that the reference
     * point becomes {@code (0,0)}.
     * <p>
     *     Uses convergence to estimate the future position of {@code coordinates} to a degree of accuracy.
     * </p>
     * <em>A reversible function that can be applied to any reference frame.</em>
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
     * <p>
     *     Uses convergence to estimate the future position of {@code coordinates} to a degree of accuracy.
     * </p>
     * <em>A reversible function that can be applied to any reference frame.</em>
     * @param referenceFrame the coordinate to use as a reference frame.
     * @param coordinates the coordinate of the target.
     * @param trajectory the direction and velocity of the target.
     * @param oldTrajectory the direction and velocity the target when the target was last calculated.
     * @param deltaTime the amount of ticks since {@code oldTrajectory} was calculated.
     * @return a coordinate to intercept.
     */
    private Vector2 circularIntercept( Vector2 referenceFrame, Vector2 coordinates, Vector2 trajectory, Vector2 oldTrajectory, long deltaTime ) {

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
     * Virtualizes bullets so that {@code this} can avoid them.
     * @param enemy the position, trajectory and velocity of an enemy.
     */
    private void virtualizeBullets( Transform enemy ){
        // TODO: 08/04/2019
    }

    /**
     * Gets the coordinates of {@code this} robot as a {@code Vector2}.
     * @return the coordinates of {@code this}.
     */
    private Vector2 getPosition() {
        return new Vector2( getX(), getY() );
    }

    /**
     * The state finite states of {@code this}.
     */
    private enum RadarStatus {

        /**
         * The robot is scanning enemies.
         * Scan 360 degrees before moving on.
         */
        SCANNING,

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
