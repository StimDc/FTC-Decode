package org.firstinspires.ftc.teamcode.shared;

public final class ShooterBallistics {

    private ShooterBallistics() {
        // Utility class
    }

    // 2600 rpm around 2 m
    public static int rpmForDistanceMeters(double distanceMeters) {
        double g = 9.81;
        double tan56 = 1.483;
        double cos56sq = 0.312;
        double k = 0.53; // efficiency
        double radius = 0.039; // shooter wheel radius in m
        double heightDiff = 0.7; // height diff in m

        double denom = 2 * cos56sq * (distanceMeters * tan56 - heightDiff);
        if (denom <= 0) {
            return 0;
        }

        double ballSpeed = Math.sqrt((g * distanceMeters * distanceMeters) / denom);
        return (int) ( 1.1 * ((60 * ballSpeed) / (2 * Math.PI * radius * k)));
    }
}
