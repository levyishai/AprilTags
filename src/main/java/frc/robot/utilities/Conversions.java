package frc.robot.utilities;

public class Conversions {
    public static final int
            MAG_TICKS = 4096,
            FALCON_TICKS = 2048;
    public static final int DEGREES_PER_REVOLUTIONS = 360;

    public static double ticksToDegrees(double magTicks) {
        return ticksToRevolutions(magTicks) * 360;
    }

    public static double ticksToRevolutions(double magTicks) {
        return magTicks / MAG_TICKS;
    }

    public static double ticksPer100MsToRotationPerSecond(double velocityPerHundredMs) {
        return falconTicksToRevolutions(velocityPerHundredMsToVelocityPerSeconds(velocityPerHundredMs));
    }

    public static double degreesToRevolutions(double degrees) {
        return degrees / DEGREES_PER_REVOLUTIONS;
    }

    public static double degreesToMagTicks(double degrees) {
        return degreesToRevolutions(degrees) * MAG_TICKS;
    }

    public static double velocityToTargetPower(double velocity, double maxSpeedMetersPerSecond) {
        return velocity / maxSpeedMetersPerSecond;
    }

    public static double velocityPerHundredMsToVelocityPerSeconds(double velocityPerHundredMs) {
        return velocityPerHundredMs * 10;
    }

    /**
     * returns the velocity in meters per second
     *
     * @param revolutions   to convert
     * @param circumference of the scope of the wheel
     */
    public static double revolutionsToMeters(double revolutions, double circumference, double gearRatio) {
        return motorRevolutionsToSystemRevolutions(revolutions, gearRatio) * circumference;
    }

    public static double motorRevolutionsToSystemRevolutions(double revolutions, double gearRatio) {
        return revolutions / gearRatio;
    }

    public static double falconTicksToRevolutions(double ticks) {
        return ticks / FALCON_TICKS;
    }
}
