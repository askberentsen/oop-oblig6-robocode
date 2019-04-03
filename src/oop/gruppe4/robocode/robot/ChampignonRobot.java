package oop.gruppe4.robocode.robot;

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

    private void aimGun(double target ){

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

        /* Predict the position of the target */
        int predictedTargetX = (targetX) + (int)(targetDX * (e.getDistance() / leadingFactor));
        int predictedTargetY = (targetY) + (int)(targetDY * (e.getDistance() / leadingFactor));

        /* Calculate the angle to the target */
        double angle = (Math.atan2( predictedTargetX, predictedTargetY ) + (2*Math.PI)) % (2*Math.PI);

        /* Take aim */
        aimGun( angle );

        /* Shoot */
        if( Utility.signedAngleDifference(getGunHeadingRadians(), angle) < 0.05 ){
            System.out.println("Within 0.5% of target");
        }
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
}
