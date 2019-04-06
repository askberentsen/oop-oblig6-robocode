package oop.gruppe4.robocode.transform;

/**
 * A class representing the transform properties of an object.
 * @author Ask Hetland Berentsen.
 */
public class Transform {

    /**
     * The position of an object.
     */
    private Vector2 position;

    /**
     * The trajectory of an object.
     */
    private Vector2 trajectory;

    /**
     * The velocity of an object.
     */
    private double velocity;

    /**
     * Class constructor.
     * @param x the x coordinate of an object.
     * @param y the y coordinate of an object.
     * @param dx the delta of {@code x}.
     * @param dy the delta of {@code y}.
     * @param velocity the velocity of an object.
     */
    public Transform( double x, double y, double dx, double dy, double velocity){
        this.position = new Vector2(x,y);
        this.trajectory = new Vector2(dx, dy);
        this.velocity  = velocity;
    }

    /**
     * Class constructor.
     * @param position the coordinate of an object.
     * @param trajectory the trajectory of an object.
     * @param velocity the velocity of an object.
     */
    public Transform( Vector2 position, Vector2 trajectory, double velocity ){
        this.position = position;
        this.trajectory = trajectory;
        this.velocity = velocity;
    }

    /**
     * Gets the position.
     * @return the coordinates of {@code this}.
     */
    public Vector2 getPosition() {
        return position;
    }

    /**
     * Gets the trajectory.
     * @return the vector {@code this} is moving in.
     */
    public Vector2 getTrajectory() {
        return trajectory;
    }

    /**
     * Gets the heading.
     * @return the heading {@code this} is moving in.
     */
    public double getHeading() {
        return trajectory.getTheta();
    }

    /**
     * Gets the velocity.
     * @return the velocity of the trajectory.
     */
    public double getVelocity() {
        return velocity;
    }
}
