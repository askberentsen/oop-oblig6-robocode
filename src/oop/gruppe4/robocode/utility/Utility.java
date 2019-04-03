package oop.gruppe4.robocode.utility;

public final class Utility {
    public static double signedAngleDifference( double alpha, double beta ){
        double naiveDifference = (alpha - beta) + Math.PI;
        double phi = Math.abs( naiveDifference ) % ( 2 * Math.PI );
        double distance = phi > Math.PI ? Math.PI - phi : phi - Math.PI;

        //TODO: 03/04/2019 These values are not consistent enough - Ask H. B.
        return alpha - beta >= 0 ? distance : -distance;
    }
}
