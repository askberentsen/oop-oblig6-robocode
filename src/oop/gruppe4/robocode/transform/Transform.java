package oop.gruppe4.robocode.transform;

public class Transform {
    private Vector2 position, trajectory;
    private double velocity;
    public Transform( double x, double y, double dx, double dy, double velocity){
        this.position = new Vector2(x,y);
        this.trajectory = new Vector2(dx, dy);
        this.velocity  = velocity;
    }

    public Vector2 getPosition() {
        return position;
    }
    public Vector2 getTrajectory() {
        return trajectory;
    }
    public double getVelocity() {
        return velocity;
    }
}
