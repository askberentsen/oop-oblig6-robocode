package oop.gruppe4.robocode.robot;

import oop.gruppe4.robocode.transform.Transform;
import oop.gruppe4.robocode.transform.Vector2;
import oop.gruppe4.robocode.utility.Utility;
import org.jetbrains.annotations.NotNull;
import robocode.*;

import java.awt.*;
import java.util.*;
import java.util.function.Predicate;

/* WARNING: BIG CODE-BLOCKS. SORRY! -Ask. */

/**
 * The main robot of the project.
 * @author Gruppe 4.
 */
public class ChampignonRobot extends AdvancedRobot {

    /* FIELDS */

    /**
     * The status of the scanner.
     * <p>
     *     {@code ChampignonRobot} uses three different states to divide its tasks into three phases.
     *     a phase will decide what actions and routines to perform and how to react to different inputs.
     * </p>
     * <p>
     *     If the robot is in the {@link RadarStatus#SCANNING} state, the robot will scan around itself,
     *     logging all the robots it found during the sweep.
     * </p>
     *
     * <p>
     *     If the robot is in the {@link RadarStatus#TARGETING} state, the robot will select a target and
     *     scan towards where the target is predicted to be.
     * </p>
     *
     * <p>
     *     If the robot is in the {@link RadarStatus#ENGAGING} state, the robot will lock the scanner
     *     onto the target so that it can be reasonably sure it has the freshest statistics of the target
     *     at all times.
     * </p>
     * @see RadarStatus
     */
    private RadarStatus status = RadarStatus.SCANNING;

    /**
     * The name of the the target.
     * <p>
     *     {@code ChampignonRobot} uses target discrimination to select a robot to target.
     *     When logging all the robots, {@code ChampignonRobot} will select a target based on
     *     some criteria and set the targeted robot as {@code targetName}.
     * </p>
     * <p>
     *     {@code targetName} is used to get the statistics from {@link #STATISTICS}.
     * </p>
     * @see #STATISTICS
     * @see #pickTarget()
     */
    private String targetName = null;

    /**
     * A counter for how many ticks the target was not found.
     * <p>
     *     When {@code ChampignonRobot} is in the {@link RadarStatus#ENGAGING} phase,
     *     it will keep track of how many ticks have passed not having scanned {@link #targetName}.
     *     If {@code targetName} was not found for a certain amount of ticks, {@code ChampignonRobot}
     *     will disengage and pick a new target.
     * </p>
     * @see #targetName
     * @see #update()
     * @see RadarStatus#ENGAGING
     */
    private int consecutiveTicksTargetNotFound = 0;

    /**
     * A list of statistics for all the robots in the game.
     * <p>
     *     Logs some relevant statistics about all the robots.
     *     If a {@code Robot} has not been scanned this update, a predicted statistic for this update
     *     will be auto-generated.
     * </p>
     * <p>
     *     If a {@code Robot} dies, or is thought to be dead, the statistics will not be cleared,
     *     but instead the statistics for the presumably dead {@code Robot} will be marked as
     *     dead, and will no longer generate predictions.
     * </p>
     * @see #targetName
     * @see #update()
     * @see #logRobot(ScannedRobotEvent)
     * @see RobotStatistics
     */
    private final HashMap<String, RobotStatistics> STATISTICS = new HashMap<>();

    /**
     * A set of {@code Robot} names scanned in one update.
     * <p>
     *     The set of names is filled up during the update, and after all the
     *     {@code Robot} names have been logged and the update is completed,
     *     the set is emptied.
     * </p>
     * @see #update()
     * @see #logRobot(ScannedRobotEvent)
     */
    private final HashSet<String> SCANNED_ROBOTS_PER_TICK = new HashSet<>();

    /**
     * A set of {@code Robot} names scanned during the {@link RadarStatus#SCANNING} phase.
     * <p>
     *     The set of names is filled up during the scanning phase, which lasts several ticks.
     *     After all the radar has completed its sweep, {@code ChampignonRobot} will
     *     update the statistics off the robots, based on whether they were scanned or not
     *     during that specific sweep.
     * </p>
     * <p>
     *     If a {@code Robot} was not found during the {@code SCANNING} phase, it will be marked as
     *     inactive. Inactive robots are robots that are not known to be dead or alive, and will be
     *     prioritized less than other robots when picking a target.
     * </p>
     * <p>
     *     If a {@code Robot} was not found during the {@code SCANNING} phase, and it had already
     *     been marked as inactive, it can confidently be declared dead.
     * </p>
     * @see #update()
     * @see #logRobot(ScannedRobotEvent)
     * @see RadarStatus#SCANNING
     */
    private final HashSet<String> SCANNED_ROBOTS_DURING_SCAN_PHASE = new HashSet<>();

    /**
     * A list of virtual bullets.
     * <p>
     *     When {@code ChampignonRobot} sees a discrepancy in the energy statistic of a {@code Robot},
     *     it can assume the robot has fired a bullet. When this happens, some virtual bullets are generated
     *     based on the transform history of {@code this}. We can assume the {@code Robot} will try to either use
     *     direct, linear or circular interception, and as such, {@code this} can avoid the positions of
     *     where these bullets would be if they were real.
     * </p>
     * @see #update()
     * @see #virtualizeBullets(Transform)
     */
    private ArrayList<Transform> virtualBullets = new ArrayList<>();


    /**
     * Class initializer.
     * <p>
     *     Sets the event priority such that {@code this} can react to killing an enemy
     *     and scanning the robots at the end of the update.
     * </p>
     * <p>
     *     Sets the gun and radar to be entirely independent.
     * </p>
     */
    @Override
    public void run() {
        /* We want to know whether a robot has been scanned or not for every update.
         * ScannedRobotEvent is set to a higher priority than StatusEvent, so that
         * we can check if a target has been scanned. */
        super.setEventPriority("RobotDeathEvent", 99);
        super.setEventPriority("ScannedRobotEvent", 98);
        super.setEventPriority("StatusEvent", 97);

        setAdjustRadarForRobotTurn( true );
        setAdjustGunForRobotTurn  ( true );
        setAdjustRadarForGunTurn  ( true );
        setBodyColor(Color.MAGENTA);
    }

    /* TICK ROUTINES */
    /**
     * The main method.
     * <p>
     *     {@code update()} is the method that will be called at the end of a update.
     * </p>
     * <p>
     *     At the end of a update, the positions of the virtual bullets and unscanned robots will
     *     be updated to a predicted position. These positions can be used when deciding where to move
     *     and where to scan.
     * </p>
     * <p>
     *     After having updated all the positions, {@code this} will decide where to move next.
     * </p>
     * <p>
     *     The main logic of {@code update()} is decided by the {@link #status}, and will react accordingly.
     * </p>
     * <p>
     *     When in the {@link RadarStatus#SCANNING} phase, this method will check whether or not
     *     the radar has completed its sweep. If it has, then it will update the statistics
     *     of all the robots that were not found during the {@code SCANNING phase}.
     * </p>
     * <p>
     *     When in the {@link RadarStatus#TARGETING} phase, this method will check whether or not
     *     the radar has scanned the target. If it has, then move to the {@code ENGAGING} phase.
     *     If however the target was not found, and the radar has completed its sweep, then we
     *     can confidently disengage with the target.
     * </p>
     * <p>
     *     When in the {@link RadarStatus#ENGAGING} phase, this method will check whether or not
     *     the radar has scanned the target. If it has <em>not</em>, then count the amount of consecutive
     *     ticks {@link #targetName} was not found. If the target has been missing for 3 or more ticks,
     *     then we can confidently disengage. Regardless whether or not the target was found during any specific
     *     update, {@code update()} will try to predict the targets position and lock the radar to an area with
     *     some leeway. For every update, the gun will aim towards the predicted position, and once the gun
     *     is sufficiently accurate, it will fire a bullet.
     *     Once the gun has fired, {@code this} begins the {@code SCANNING} phase again while waiting for the gun
     *     to cool down.
     * </p>
     * <p>
     *     At the very end of a update, {@code SCANNED_ROBOTS_PER_TICK} will be cleared.
     * </p>
     * @see #targetName
     * @see #STATISTICS
     * @see #virtualBullets
     * @see #SCANNED_ROBOTS_PER_TICK
     * @see #SCANNED_ROBOTS_DURING_SCAN_PHASE
     * @see #aimGun()
     * @see #disengage()
     * @see #lockScanner()
     * @see RadarStatus
     */
    private void update() {

        /* Virtual bullet routine. */
        updateVirtualBullets();

        /* History routine. */
        updateStatistics();

        /* Movement routine. */
        updateMovement();

        /* Scanner routine. */
        switch ( status ) {

            case SCANNING: {
                updateScanningState();
                break;
            }
            case TARGETING: {
                updateTargetingState();
                break;
            }
            case ENGAGING: {
                updateEngagingState();
                break;
            }
        }

        /* Reset the counter */
        SCANNED_ROBOTS_PER_TICK.clear();
    }

    /**
     * Updates the positions of the virtual bullets.
     * @see #virtualBullets
     * @see #virtualizeBullets(Transform)
     */
    private void updateVirtualBullets() {
        // TODO: 10/04/2019
        for( Transform virtualBullet : virtualBullets ) {
            virtualBullet.update();
            if( !virtualBullet.getPosition().isContained( 0, 0,getBattleFieldWidth(), getBattleFieldHeight() )){
                virtualBullets.remove( virtualBullet );
            }
        }
    }

    /**
     * Updates the positions of the other robots.
     * @see #STATISTICS
     * @see #SCANNED_ROBOTS_PER_TICK
     */
    private void updateStatistics() {
        STATISTICS.forEach( (name, statistics) -> {
            /* Assume that robots that were not scanned this update are moving linearly. */
            if( !SCANNED_ROBOTS_PER_TICK.contains( name ) && statistics.isAlive() ){
                // TODO: 09/04/2019 do predictions in ChampignonRobot instead of in robotStatistics. Account for boundaries.
                statistics.predict();
            }
        });
    }

    /**
     * The method to run every tick the {@link #status} is set to {@link RadarStatus#SCANNING}
     */
    private void updateScanningState() {
        if( getRadarTurnRemainingRadians() == 0.0 ){

            /* Disengage all enemies that were not found during the scan phase. */
            STATISTICS.forEach( (name, statistics) -> {
                if( !SCANNED_ROBOTS_DURING_SCAN_PHASE.contains(name) ){

                    /* If a robot that was not found during this sweep, deactivate it. */
                    if( statistics.isActive() ) {
                        statistics.setActive(false);
                    }
                    /* If a robot was not found this sweep, and the amount of alive robots in
                     * in the game is different to the amount of robots that have been counted as
                     * alive, assume dead.
                     */
                    else if( getOthers() != getAliveRobots().size()){
                        statistics.setAlive(false);
                    }
                }
            });

            beginTargetPhase();
        }
    }

    /**
     * The method to run every tick the {@link #status} is set to {@link RadarStatus#TARGETING}
     */
    private void updateTargetingState() {
        aimGun();

        if( SCANNED_ROBOTS_PER_TICK.contains(targetName) ){
            beginEngagePhase();
        }
        else if( getRadarTurnRemainingRadians() == 0.0 ){
            //enemy was not where expected.
            disengage();
        }
    }

    /**
     * The method to run every tick the {@link #status} is set to {@link RadarStatus#ENGAGING}
     */
    private void updateEngagingState(){
        if( !SCANNED_ROBOTS_PER_TICK.contains(targetName) ){
            consecutiveTicksTargetNotFound++;
            if( consecutiveTicksTargetNotFound > 2 ){
                disengage();
                return;
            }
        }
        else{
            consecutiveTicksTargetNotFound = 0;
        }

        lockScanner();

        aimGun();

        final double ACCEPTABLE_ACCURACY = Math.PI / 180;
        if( getGunHeat() == 0 && getGunTurnRemainingRadians() < ACCEPTABLE_ACCURACY ){

            // TODO: 09/04/2019 save the bullet information so we can track which bullet missed which target.
            Bullet bullet = setFireBullet( Math.min( getEnergy(), Rules.MAX_BULLET_POWER ) );
            beginScanPhase();
        }
    }

    /* TARGET DISCRIMINATION */

    /**
     * Decides which {@code Robot} to target.
     * <p>
     *     Compares all the alive entries of {@link #STATISTICS} and finds a
     *     {@code Robot} to target.
     * </p>
     * @see #targetName
     * @see #STATISTICS
     * @see RobotStatistics
     */
    private void pickTarget() {

        /* The name of the highest rated target. */
        String targetName = null;

        /* The statistics of the highetst rated target. */
        RobotStatistics targetStatistics = null;

        /* For every entry in STATISTICS, compare it to the highest rated target. */
        for( Map.Entry<String,RobotStatistics> entry : STATISTICS.entrySet() ) {

            final String ROBOT_NAME = entry.getKey();
            final RobotStatistics ROBOT_STATISTICS = entry.getValue();

            /* Compare statistics. Ignore the robots that are presumed dead. */
            if( ROBOT_STATISTICS.isAlive() ){

                /* If no target has been picked before, pick the first one. */
                if( targetStatistics == null ){
                    targetName = ROBOT_NAME;
                    targetStatistics = ROBOT_STATISTICS;
                    continue;
                }

                /* A predicate for picking the most relevant comparision. */
                final Predicate<Integer[]> PRIORITIZED_COMPARATOR = list -> {
                    for( int n : list){
                        if( n > 0 ) return true;
                        else if( n < 0) return false;
                    }
                    return false;
                };
                /* Robot is active and target is inactive*/
                final int ACTIVITY_DIFFERENCE = Boolean.compare( ROBOT_STATISTICS.isActive(), targetStatistics.isActive());

                /* Robot is more agression than the target */
                final int AGGRESSION_DIFFERENCE = Double.compare( ROBOT_STATISTICS.getAggression(), targetStatistics.getAggression() );

                /* Robot has more energy than the target */
                final int ENERGY_DIFFERENCE = Double.compare(
                        ROBOT_STATISTICS.getLast().getEnergy(),
                        targetStatistics.getLast().getEnergy()
                );

                /* Robot is closer than the target */
                final int DISTANCE_DIFFERENCE = Double.compare(
                        targetStatistics.getLast().getPosition().subtract( this.getPosition() ).getScalar(),
                        ROBOT_STATISTICS.getLast().getPosition().subtract( this.getPosition() ).getScalar()
                );

                final Integer[] LIST_OF_DIFFERENCES = new Integer[]{
                        AGGRESSION_DIFFERENCE,
                        ACTIVITY_DIFFERENCE,
                        ENERGY_DIFFERENCE,
                        DISTANCE_DIFFERENCE
                };

                /* If the robot is rated higher than the currently highest rated target,
                 * set that robot as the highest rated target.
                 */
                if( PRIORITIZED_COMPARATOR.test( LIST_OF_DIFFERENCES ) ){
                    targetName = ROBOT_NAME;
                    targetStatistics = ROBOT_STATISTICS;
                }
            }
        }
        /* Update the target-name */
        this.targetName = targetName;
    }

    /**
     * Disengages from the target.
     * <p>
     *     Sets the target to inactive, picks a new target and starts the {@link RadarStatus#SCANNING}
     *     phase.
     * </p>
     */
    private void disengage() {
        STATISTICS.get( targetName ).setActive( false );
        pickTarget();
        beginScanPhase();
    }

    /**
     * Logs the statistics of a scanned {@code Robot}.
     * @param e a scanned {@code Robot}.
     * @see #STATISTICS
     * @see RobotStatistics
     * @see RobotStatistics.Statistic
     */
    private void logRobot(@NotNull ScannedRobotEvent e ) {

        final String ROBOT_NAME = e.getName();
        
        //if( !STATISTICS.containsKey( ROBOT_NAME ) ) STATISTICS.put( ROBOT_NAME, new RobotStatistics(30) );
        STATISTICS.putIfAbsent( ROBOT_NAME, new RobotStatistics(30) );

        final RobotStatistics ROBOT_STATISTICS = STATISTICS.get( ROBOT_NAME );

        /* If the scanned robot is alive, set to active. */
        if( ROBOT_STATISTICS.isAlive() ){
            ROBOT_STATISTICS.setActive(true);
        }
        /* Robot died this turn, updating information not necessary.  */
        else return;

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
        final RobotStatistics.Statistic STATISTIC = new RobotStatistics.Statistic(
                ROBOT_X,
                ROBOT_Y,
                ROBOT_DX,
                ROBOT_DY,
                ROBOT_VELOCITY,
                TIMESTAMP,
                ENERGY
        );

        ROBOT_STATISTICS.add( STATISTIC );
    }

    /* ENGAGEMENT METHODS */

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

        final RobotStatistics TARGET_STATISTICS = STATISTICS.get(targetName);
        final RobotStatistics.Statistic CURRENT_TARGET_STAT = TARGET_STATISTICS.getLast();
        final RobotStatistics.Statistic PREVIOUS_TARGET_STAT = TARGET_STATISTICS.getPrevious();

        final Vector2 TARGET_COORDINATES = CURRENT_TARGET_STAT.getPosition();
        final Vector2 TARGET_TRAJECTORY = CURRENT_TARGET_STAT.getTrajectory();
        final Vector2 OLD_TARGET_TRAJECTORY = PREVIOUS_TARGET_STAT.getTrajectory();
        long DELTA_TIME = CURRENT_TARGET_STAT.getTimeStamp() - PREVIOUS_TARGET_STAT.getTimeStamp();

        final double DELTA_ANGLE = Utility.signedAngleDifference(
                OLD_TARGET_TRAJECTORY.getTheta(),
                TARGET_TRAJECTORY.getTheta()
        );

        Vector2 predictedPosition;

        /* Direct interception */
        if( TARGET_TRAJECTORY.getScalar() == 0.0 || TARGET_TRAJECTORY.add( OLD_TARGET_TRAJECTORY ).getScalar() < 0.5){
            predictedPosition = TARGET_COORDINATES;
        }
        /* Linear interception. */
        else if( Math.abs(DELTA_ANGLE) < 0.05 ){
            predictedPosition = linearIntercept(
                    this.getPosition(),
                    TARGET_COORDINATES,
                    TARGET_TRAJECTORY
            );
        }
        /* Circular interception. */
        else{
            predictedPosition = circularIntercept(
                    this.getPosition(),
                    TARGET_COORDINATES,
                    TARGET_TRAJECTORY,
                    OLD_TARGET_TRAJECTORY,
                    DELTA_TIME );
        }

        final double BOUNDS = 15.0;
        final Vector2 RESTRICTED_PREDICTED_POSITION = new Vector2(
                Utility.limit( predictedPosition.getX(), BOUNDS, getBattleFieldWidth() - BOUNDS ),
                Utility.limit( predictedPosition.getY(), BOUNDS, getBattleFieldHeight() - BOUNDS )
        );

        final Vector2 RELATIVE_PREDICTED_POSITION = RESTRICTED_PREDICTED_POSITION.subtract( this.getPosition() );
        final double ANGLE_TO_TARGET = RELATIVE_PREDICTED_POSITION.getTheta();

        final double THETA = Utility.signedAngleDifference( getGunHeadingRadians(), ANGLE_TO_TARGET );
        setTurnGunRightRadians( THETA );
    }

    /**
     * Locks the scanner to the area around the target {@code Robot}.
     * <p>
     *     Calculates the maximum angle that the radar can sweep in one update and constrains it
     *     with some uncertainty factor so that if the target moves outside of the bounds, the radar
     *     has enough room to overshoot the target.
     * </p>
     * @see #getTargetStatistics()
     */
    private void lockScanner() {

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

    /* MOVEMENT METHODS */

    /**
     * TODO: 10/04/2019
     */
    public void updateMovement() {

    }

    /**
     * Virtualizes bullets so that {@code this} can avoid them.
     * @param enemy the position, trajectory and velocity of an enemy.
     */
    private void virtualizeBullets( Transform enemy ) {
        // TODO: 08/04/2019
    }

    /* STATUS MANAMGEMENT */

    /**
     * Initializes the {@link RadarStatus#SCANNING} phase.
     * <p>
     *     Sets the radar to scan {@code 360} degrees.
     * </p>
     * @see #status
     * @see #update()
     * @see RadarStatus#SCANNING
     */
    private void beginScanPhase() {
        status = RadarStatus.SCANNING;

        /* Scan 720 degrees in case target has moved. */
        final double THETA = Math.PI * 2.0;
        setTurnRadarRightRadians( THETA );
    }

    /**
     * Initializes the {@code TARGETING} phase.
     * Rotate the scanner to the targets expected position.
     * Intentionally overshoots based on an uncertainty factor to ensure the target will most likely
     * be found. If the target is not found during this phase, we can confidently disengage the target.
     * @see #onStatus(StatusEvent)
     * @see RadarStatus#TARGETING
     */
    private void beginTargetPhase() {
        status = RadarStatus.TARGETING;
        pickTarget();
        if( targetName == null ){
            /* No targets are available. */
            beginScanPhase();
            return;
        }

        /* Calculate the angle to move the radar. */
        final Vector2 RELATIVE_POSITION = getTargetStatistics().getPosition().subtract( this.getPosition() );
        final double ANGULAR_DIFFERENCE = Utility.signedAngleDifference( getRadarHeadingRadians(), RELATIVE_POSITION.getTheta() );
        final double UNCERTAINTY_FACTOR = 1.5;
        final int    SIGN = ANGULAR_DIFFERENCE >= 0.0 ? 1 : -1;
        final double THETA = Math.PI * SIGN * UNCERTAINTY_FACTOR;

        setTurnRadarRightRadians( THETA );
    }

    /**
     * Initializes the {@code ENGAGING} phase.
     * Rotate the scanner to overshoot the targets expected position.
     * When the scanner overshoots the target, it will most likely produce a {@code ScannedRobotEvent}.
     * @see #onStatus(StatusEvent)
     * @see RadarStatus#ENGAGING
     */
    private void beginEngagePhase() {
        status = RadarStatus.ENGAGING;

        /* Calculate the angle to move the radar. */
        final Vector2 RELATIVE_POSITION = getTargetStatistics().getPosition().subtract( this.getPosition() );
        final double ANGULAR_DIFFERENCE = Utility.signedAngleDifference( getRadarHeadingRadians(), RELATIVE_POSITION.getTheta() );
        final double UNCERTAINTY_FACTOR = 1.1;
        final double THETA = ANGULAR_DIFFERENCE * UNCERTAINTY_FACTOR;

        setTurnRadarRightRadians( THETA );
    }

    /* GET METHODS */

    /**
     * Gets the coordinates of {@code this} as a {@code Vector2}.
     * @return the coordinates of {@code this}.
     */
    private Vector2 getPosition() {
        return new Vector2( getX(), getY() );
    }

    /**
     * Gets the latest statistics of the target. Either fresh or predicted.
     * @return the statistics of the target this update.
     * @throws NullPointerException if the target has not been initialized.
     * @see #STATISTICS
     */
    private RobotStatistics.Statistic getTargetStatistics() {
        return STATISTICS.get(targetName).getLast();
    }

    /**
     * Gets a set of all all the robots that are alive.
     * @return a {@code HashSet} of {@code Robot} names.
     */
    private HashSet<String> getAliveRobots() {
        HashSet<String> aliveRobots = new HashSet<>();
        for(Map.Entry<String,RobotStatistics> entry : STATISTICS.entrySet()){
            if( entry.getValue().isAlive() ) aliveRobots.add(entry.getKey());
        }
        return aliveRobots;
    }

    /* EVENT HANDLERS */

    /**
     * This method is called when this robot kills another.
     * <p>
     *     Disengages if the killed {@code Robot} was targeted.
     * </p>
     * <p>
     *     Sets the {@code RobotStatistics} of the killed {@code Robot} to dead.
     * </p>
     * @see #STATISTICS
     * @see #disengage()
     */
    @Override
    public void onRobotDeath( RobotDeathEvent e ) {
        /* Disengage and clear from history. */
        if( e.getName().equals(targetName) ) disengage();
        STATISTICS.get(e.getName()).setAlive(false);
    }

    /**
     * This method is called when the status is updated.
     * <p>
     *     Called every update.
     * </p>
     * @param e a {@code StatusEvent}.
     */
    @Override
    public void onStatus( StatusEvent e ) {
        /* onStatus is called before run() has completed.
         * Ensure that the scanning phase has started before performing any actions.
         */
        if( e.getTime() == 0 ){
            beginScanPhase();
        }

        /* Main method. */
        update();
    }
    
    /**
     * This method is called when your robot sees another robot, i.e. when the robot's radar scan "hits" another robot.
     * <p>
     *     Assumes the enemy has fired a bullet if the energy of the scanned robot is
     *     discrepant to the expected energy level.
     * </p>
     */
    @Override
    public void onScannedRobot( ScannedRobotEvent e ) {

        /* Log the statistics of the scanned robot. */
        logRobot(e);
        final String ROBOT_NAME = e.getName();

        /* Add robot to the list of robots found this sweep. */
        SCANNED_ROBOTS_PER_TICK.add( ROBOT_NAME );

        if( status == RadarStatus.SCANNING ){
            SCANNED_ROBOTS_DURING_SCAN_PHASE.add(ROBOT_NAME);
        }

        final RobotStatistics ROBOT_STATISTICS = STATISTICS.get( ROBOT_NAME );

        final RobotStatistics.Statistic CURRENT_TARGET_STAT = ROBOT_STATISTICS.getLast();
        final RobotStatistics.Statistic PREVIOUS_TARGET_STAT = ROBOT_STATISTICS.getPrevious();

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
        STATISTICS.get(e.getName()).addAggression(e.getPower());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void onBulletHit( BulletHitEvent e ) {
        STATISTICS.get(e.getName()).addHit();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void onBulletMissed(BulletMissedEvent event) {
        // TODO: 10/04/2019
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void onHitRobot(HitRobotEvent event) {
        // TODO: 10/04/2019
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void onHitWall( HitWallEvent e ) {
        // TODO: 09/04/2019
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
