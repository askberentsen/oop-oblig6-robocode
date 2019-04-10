package oop.gruppe4.robocode.robot;

import oop.gruppe4.robocode.transform.Transform;
import oop.gruppe4.robocode.transform.Vector2;
import oop.gruppe4.robocode.utility.Utility;
import org.jetbrains.annotations.NotNull;

import java.util.LinkedList;

/**
 * {@code RobotStatistics} keeps track of a robots previous {@code Transform}s.
 * @author Ask Hetland Berentsen.
 */
public class RobotStatistics {

    private Status status = Status.ACTIVE;
    private String reasonForBeingTargeted;

    private int misses = 0;
    private int hits = 0;
    private double aggression = 0;

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

    public void setReason( String reason ){
        this.reasonForBeingTargeted = reason;
    }
    public String getReason(){
        return this.reasonForBeingTargeted;
    }
    public void setStatus( Status status ){
        this.status = status;
    }

    public Status getStatus(){
        return this.status;
    }

    public boolean isActive(){
        return status.getValue() >= 2;
    }

    public boolean isAlive() {
        return status.getValue() >= 1;
    }

    public boolean isReitred() {
        return status == Status.RETIRED;
    }

    public double getAggression() {
        return aggression;
    }

    public void addAggression( double amount ){
        this.aggression += amount;
    }

    public int getHits() {
        return hits;
    }

    public void addHit() {
        this.hits++;
    }

    public int getMisses() {
        return misses;
    }

    public void addMiss() {
        this.misses++;
    }

    public void predict() {
        Statistic last = getLast();
        Statistic previous = getPrevious();

        Vector2 position = last.getPosition().add( last.getTrajectory() );

        Vector2 trajectory = last.getNormalizedTrajectory();

        double velocity = last.getVelocity();

        long timeStamp = last.timeStamp + 1;
        double energyDelta = last.energy;

        Statistic generated = new Statistic(
                position,
                trajectory,
                velocity,
                timeStamp,
                energyDelta
        );
        generated.predicted = true;
        history.add( generated );
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

        long timeStampDelta = last.timeStamp - previous.timeStamp;
        double energyDelta = last.energy - previous.energy;

        return new Statistic( positionDelta, trajectoryDelta, velocityDelta, timeStampDelta, energyDelta );
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

    /**
     * Statistics of a robot.
     */
    public static class Statistic extends Transform {

        /**
         * A timestamp.
         */
        private long timeStamp;

        /**
         * The energy left.
         */
        private double energy;

        private boolean predicted = false;

        /**
         * Class constructor.
         * @param position the coordinates of this logging.
         * @param trajectory the trajectory of this logging.
         * @param velocity the velocity of this logging.
         * @param timeStamp the timestamp of this logging.
         * @param energy the energy of this logging.
         */
        public Statistic( Vector2 position, Vector2 trajectory, double velocity, long timeStamp, double energy ){
            super( position, trajectory, velocity);
            this.timeStamp = timeStamp;
            this.energy = energy;
        }

        /**
         * Class constructor.
         * @param x the x coordinate of this logging.
         * @param y the y coordinate of this logging.
         * @param dx the x trajectory of this logging.
         * @param dy the y trajectory of this logging.
         * @param velocity the velocity of this logging.
         * @param timeStamp the timestamp of this logging.
         * @param energy the energy of this logging.
         */
        public Statistic( double x, double y, double dx, double dy, double velocity, long timeStamp, double energy ){
            super(x,y,dx,dy,velocity);
            this.timeStamp = timeStamp;
            this.energy = energy;
        }

        /**
         * Gets the timestamp.
         * @return a timestamp.
         */
        public long getTimeStamp(){
            return timeStamp;
        }

        /**
         * Gets the energy.
         * @return the amount of energy left.
         */
        public double getEnergy() {
            return energy;
        }

        public boolean isPredicted() {
            return predicted;
        }

        public String toString(){
            return String.format("Statistic: %s -> %s @ %d", getPosition(), getTrajectory(), timeStamp);
        }
    }

    public enum Status {
        ACTIVE(2),
        UNKNOWN(1),
        RETIRED(0);
        private final int VALUE;
        Status( int value ){
            this.VALUE = value;
        }
        public int getValue(){
            return this.VALUE;
        }
    }
}
