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
     * A counter for how many ticks the target was not found.
     */
    private int consecutiveTicksTargetNotFound = 0;

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
     * The name of the the target.
     */
    private String targetName;

    /**
     * A list of names of the robots scanned in one scanning sweep.
     */
    private ArrayList<String> scannedRobotsPerTick = new ArrayList<>();

    private ArrayList<String> scannedRobotsDuringScanPhase = new ArrayList<>();

    /**
     * Main method of {@code this}.
     */
    @Override
    public void run() {
        /* We want to know whether a robot has been scanned or not for every tick.
         * ScannedRobotEvent is set to a higher priority than StatusEvent, so that
         * we can check if a target has been scanned. */
        super.setEventPriority("RobotDeathEvent", 99);
        super.setEventPriority("ScannedRobotEvent", 98);
        super.setEventPriority("StatusEvent", 97);

        setAdjustRadarForRobotTurn( true );
        setAdjustGunForRobotTurn  ( true );
        setAdjustRadarForGunTurn  ( true );
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
        // TODO: 09/04/2019 Scanner should sweep back and forth between two angles.
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
     */
    private void disengage(){
        System.out.println(String.format("Disengaging %s: Could not find target.", targetName ));
        targetName = null;
        beginScanPhase();
    }

    /**
     * Update on every tick.
     * @param e the status event.
     */
    @Override
    public void onStatus( StatusEvent e ) {

        final long TIMESTAMP = e.getTime();

        /* Virtual bullet routine. */
        for( Transform virtualBullet : virtualBullets ) {
            // TODO: 09/04/2019 virtual bullets routine.
            virtualBullet.update();
            if( !virtualBullet.getPosition().isContained( 0, 0,getBattleFieldWidth(), getBattleFieldHeight() )){
                virtualBullets.remove( virtualBullet );
            }
        }

        /* History routine. */
        history.forEach( (name, statistics) -> {
            /* Assume that robots that were not scanned this tick are moving linearly. */
            if( !scannedRobotsPerTick.contains( name ) ){
                // TODO: 09/04/2019 do predictions in ChampignonRobot instead of in robotStatistics. Account for boundaries.
                statistics.predict();
            }
        });


        /* Movement routine. */
        // TODO: 09/04/2019 update self.

        /* Scanner routine. */

        switch ( status ) {
            case SCANNING: {
                /* Scan 360 degrees to log the transforms of every enemy.
                 * When scanner is done, aim the scanner to where the enemy was last seen.
                 * Begin targeting phase.
                 */
                if( getRadarTurnRemainingRadians() == 0.0 ){
                    //finished scanning

                    /* Disengage all enemies that were not found during the scan phase. */
                    history.forEach( (name, statistics)->{
                        if( !scannedRobotsDuringScanPhase.contains(name) ){
                            statistics.setActive( false );
                        }
                    });

                    /* Begin targeting. */
                    if( targetName != null ) {
                        beginTargetPhase();
                    }
                    else{
                        beginScanPhase();
                    }
                }
                break;
            }
            case TARGETING: {

                if( scannedRobotsPerTick.contains(targetName) ){
                    beginEngagePhase();
                }
                else if( getRadarTurnRemainingRadians() == 0.0 ){
                    disengage();
                }
                /* Aim the scanner to the targets last known position.
                 * If the target was not found at the last known position, continue to scan for
                 * 180 degrees.
                 * If the enemy was not found at all, disengage. -> enter Scanning phase.
                 *
                 * if the enemy was found, enter engagement phase.
                 *
                 * Aim gun to the targets predicted position based on the predicted current position ( update via history )
                 */
                break;
            }
            case ENGAGING: {

                final double ANGLE_TO_TARGET = getTargetStatistics().getPosition().subtract( this.getPosition() ).getTheta();
                final double UNCERTAINTY_FACTOR = 0.75;
                final double DEVIATION = Rules.RADAR_TURN_RATE_RADIANS * UNCERTAINTY_FACTOR;
                final double LOWER_BOUNDS = ANGLE_TO_TARGET - (DEVIATION/2);
                final double UPPER_BOUNDS = ANGLE_TO_TARGET + (DEVIATION/2);

                /* The angle to the lower bounds. */
                final double ALPHA = Utility.signedAngleDifference( getRadarHeadingRadians(), LOWER_BOUNDS );

                /* The angle to the upper bounds. */
                final double BETA = Utility.signedAngleDifference( getRadarHeadingRadians(), UPPER_BOUNDS );

                /* Set the scanner to either ALPHA or BETA, whichever has the biggest magnitude. */
                setTurnRadarRightRadians( Math.abs( ALPHA ) > Math.abs( BETA ) ? ALPHA : BETA );

                //beginScanPhase();

                /* Lock the scanner to the target.
                 * Aim the gun to the targets predicted position.
                 * Shoot at enemy after having scanned for at least 2 ticks, gun is cooled down, target is close enough
                 * and if the gun is aiming at the predicted position with a reasonable degree of accuracy (less than 5 pixels).
                 *
                 * If the enemy was lost for 450 degrees, disengage.
                 */
                break;
            }
        }

        /* Reset foundTarget & scannedRobot */
        scannedRobotsPerTick.clear();
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
     */
    @Override
    public void onScannedRobot( ScannedRobotEvent e ) {

        /* Log the statistics of the scanned robot. */
        logTarget(e);
        final String ROBOT_NAME = e.getName();
        /* Add robot to the list of robots found this sweep. */
        scannedRobotsPerTick.add( ROBOT_NAME );

        if( status == RadarStatus.SCANNING && !scannedRobotsDuringScanPhase.contains(ROBOT_NAME) ){
            scannedRobotsDuringScanPhase.add(ROBOT_NAME);
        }

        /* Get the statistics of the target (or the scanned robot if no robot was targeted) */
        final RobotStatistics STATISTICS = history.get( targetName );

        final RobotStatistics.Statistic CURRENT_TARGET_STAT = STATISTICS.getLast();
        final RobotStatistics.Statistic PREVIOUS_TARGET_STAT = STATISTICS.getPrevious();

        /* Initiate virtualized bullets routine */
        if (CURRENT_TARGET_STAT.getEnergy() - PREVIOUS_TARGET_STAT.getEnergy() < 0) {

            /* Virtual bullets routine. */
            virtualizeBullets(CURRENT_TARGET_STAT);
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
        /* Disengage and clear from history. */
        if( e.getName().equals(targetName) ) disengage();
        history.remove( e.getName() );
    }

    /**
     * Initializes the {@code SCANNING} phase.
     * @see RadarStatus#SCANNING
     */
    private void beginScanPhase() {
        System.out.println("SCANNING...");
        status = RadarStatus.SCANNING;
        /* Scan 720 degrees in case target has moved. */
        setTurnRadarRightRadians( 2 * Math.PI );
    }

    /**
     * Initializes the {@code TARGETING} phase.
     * @see RadarStatus#TARGETING
     */
    private void beginTargetPhase() {
        System.out.println("TARGETING " + targetName);
        Vector2 position = getTargetStatistics().getPosition().subtract( this.getPosition() );
        double theta = Utility.signedAngleDifference( getRadarHeadingRadians(), position.getTheta() );
        setTurnRadarRightRadians( theta >= 0.0 ? Math.PI : -Math.PI );
        status = RadarStatus.TARGETING;
    }

    /**
     * Initializes the {@code ENGAGING} phase.
     * @see RadarStatus#ENGAGING
     */
    private void beginEngagePhase() {
        System.out.println("ENGAGING " + targetName);
        Vector2 position = getTargetStatistics().getPosition().subtract( this.getPosition() );
        double theta = Utility.signedAngleDifference( getRadarHeadingRadians(), position.getTheta() );
        setTurnRadarRightRadians( theta * 1.1 );
        status = RadarStatus.ENGAGING;
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
     * Gets the latest statistics of the target. Either fresh or predicted.
     * @return the statistics of the target this tick.
     */
    private RobotStatistics.Statistic getTargetStatistics(){
        return history.get(targetName).getLast();
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
         *
         */
        TARGETING,

        /**
         * Actively engaging a locked-on target.
         * Aim and shoot.
         */
        ENGAGING
    }
}
