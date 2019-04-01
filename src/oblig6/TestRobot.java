package oblig6;
import oblig6.vector.Vector;
import robocode.AdvancedRobot;
import robocode.*;

public class TestRobot extends AdvancedRobot {

    private int enemy_x=0, enemy_y=0;
    private double lead_factor=1, enemy_dx=0, enemy_dy=0;
    private Vector enemy_delta;
    private double firepower = 1;

    @Override
    public void run(){

        while(true){
            turnGunLeft(360);
            execute();
        }

    }

    @Override
    public void onScannedRobot( ScannedRobotEvent e ){
        getEnemyPosition(e);
    }

    private void getEnemyPosition( ScannedRobotEvent e ){

        /* Calculate the angle to find the enemy coordinates from */
        double angle =  Math.toRadians((getHeading() + e.getBearing()) % 360);

        /* Calculate the enemy coordinates */
        enemy_x = (int)(getX() + Math.sin(angle) * e.getDistance());
        enemy_y = (int)(getY() + Math.cos(angle) * e.getDistance());

        /* Calculate the angle to find the enemy delta from */
        double lead_angle = Math.toRadians( e.getHeading() );

        /* Calculate the enemy delta coordinates */
        enemy_dx = (Math.sin(lead_angle) * e.getVelocity());
        enemy_dy = (Math.cos(lead_angle) * e.getVelocity());

        enemy_delta = new Vector(enemy_x, enemy_dy);

        /* Calculate factor to lead the shots with */
        //TODO: Figure out how to calculate the lead factor properly.
        lead_factor =  (20-(3*firepower)) /  e.getDistance();

    }
}
