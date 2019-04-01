package oblig6.vector;

public class Vector {
    private final double x, y;
    public Vector( double x, double y ){
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Vector add( double dx, double dy ){
        return new Vector( this.x + dx, this.y + dy );
    }

    public Vector multiply( double scalar){
        return new Vector( this.x*scalar, this.y*scalar);
    }
}
