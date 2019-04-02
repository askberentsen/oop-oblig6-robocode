package oop.gruppe4.robocode.robot;

import robocode.AdvancedRobot;
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
    private double leadingFactor;

    /**
     * Main method of this {@code ChampignonRobot}.
     */
    @Override
    public void run() {
    }

    private void targetRobot(ScannedRobotEvent e){
        //TODO: Calculate the values of the target

        /* Calculate the absolute bearing of the target. */
        double absoluteBearing = Math.toRadians( ( getHeading() + e.getBearing() ) % 360 );

        /* Calculate the coordinates. */
        targetX = (int)( getX() + Math.sin( absoluteBearing ) * e.getDistance() );
        targetY = (int)( getY() + Math.cos( absoluteBearing ) * e.getDistance() );

        /* Calculate the absolute angle the enemy is moving towards. */
        double angle = e.getHeadingRadians();

        /* Calculate the relative later coordinates */
        targetDX = ( Math.sin( angle ) * e.getVelocity() );
        targetDY = ( Math.cos( angle ) * e.getVelocity() );

        //TODO: Calculate the scalar to use when leading the shot
    }
    private void aim(ScannedRobotEvent e){
        //Aim to enemy, lead the shot.
        //if gun is locked on the predicted position, shoot
    }
    private void lockOn(ScannedRobotEvent e){
        //adjust radar to keep pointing to enemy
    }

    @Override
    public void onScannedRobot( ScannedRobotEvent e ) {
        targetRobot(e); //targets the robot coordinates and vector of enemy
        lockOn(e);
        aim(e);
    }
}
