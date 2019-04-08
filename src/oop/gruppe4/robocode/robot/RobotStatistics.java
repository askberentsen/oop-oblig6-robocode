package oop.gruppe4.robocode.robot;

import oop.gruppe4.robocode.transform.Transform;
import oop.gruppe4.robocode.transform.Vector2;
import org.jetbrains.annotations.NotNull;

import java.util.LinkedList;

/**
 * {@code RobotStatistics} keeps track of a robots previous {@code Transform}s.
 * @author Ask Hetland Berentsen.
 */
public class RobotStatistics {

    /**
     * The status.
     */
    private boolean alive = true;

    /**
     * The amount of energy left.
     */
    private double energy = 100;

    /**
     * The history of Transforms.
     */
    private final LinkedList<Statistic> history = new LinkedList<>();

    /**
     * The amount of entries to log at once.
     */
    private final int capacity;

    /**
     * Class constructor.
     * @param capacity the amount of entries to log at once.
     */
    public RobotStatistics( int capacity ){
        this.capacity = capacity;
    }

    /**
     * Adds a {@code Transform} to the history.
     * If the amount of entries exceeds that of the capacity, the first
     * entry is removed.
     * @param stat a {@code Transform}.
     */
    public void add( @NotNull Statistic stat ){

        /* If the size of history is larger than the capacity, remove the first entries */
        while( history.size() >= capacity ) history.removeFirst();

        history.add( stat );
    }

    /**
     * Sets the energy.
     * @param energy the amount of energy left.
     */
    public void setEnergy( double energy ){
        this.energy = energy;
    }

    /**
     * Sets the status to retired.
     */
    public void retire(){
        this.alive = false;
    }

    /**
     * Gets the energy.
     * @return the amount of energy left.
     */
    public double getEnergy() {
        return energy;
    }

    /**
     * Checks if {@code this} is alive
     * @return {@code true} if {@code this} is alive.
     *         {@code false} otherwise.
     */
    public boolean isAlive(){
        return alive;
    }

    /**
     * Gets the last entry in the history.
     * @return the last {@code Transform} in the history.
     * @throws NullPointerException if {@code history} is empty.
     * @see #history
     */
    public Statistic getLast(){
        return history.getLast();
    }

    /**
     * Calculates the difference between the last and the next to last
     * {@code Transform} in the history.
     * @return the delta of two {@code Transform}s.
     * @throws NullPointerException if {@code history} is empty.
     * @see #history
     */
    public Statistic getDelta(){
        /* If getDelta() is called before two entries exists, the delta is just the last index */
        if( history.size() == 1) return history.getLast();

        Statistic last = history.getLast();
        Statistic previous = history.get(history.size()-2);

        Vector2 positionDelta = last.getPosition().subtract(previous.getPosition());
        Vector2 trajectoryDelta = last.getTrajectory().subtract(previous.getTrajectory());
        double velocityDelta = last.getVelocity() - previous.getVelocity();
        long timeStampDelta = last.getTimeStamp() - previous.getTimeStamp();

        return new Statistic( positionDelta, trajectoryDelta, velocityDelta, timeStampDelta );
    }

    public Statistic getPrevious(){
        if( history.size() == 1) return history.getLast();

        return history.get(history.size()-2);
    }

    /**
     * Gets a specific entry in the history.
     * @param index the index to fetch from.
     * @return
     * @throws IndexOutOfBoundsException if the index is out of range
     *         ({@code index < 0 || index >= history.size()})
     * @see #history
     */
    public Statistic get( int index ){
        return history.get( index );
    }

    public static class Statistic extends Transform {
        private long timeStamp;
        public Statistic( Vector2 position, Vector2 trajectory, double velocity, long timeStamp ){
            super( position, trajectory, velocity);
            this.timeStamp = timeStamp;
        }
        public Statistic( double x, double y, double dx, double dy, double velocity, long timeStamp){
            super(x,y,dx,dy,velocity);
            this.timeStamp = timeStamp;
        }
        public long getTimeStamp(){
            return timeStamp;
        }

        public String toString(){
            return String.format("Statistic: %s -> %s @ %d", getPosition(), getTrajectory(), timeStamp);
        }
    }
}
