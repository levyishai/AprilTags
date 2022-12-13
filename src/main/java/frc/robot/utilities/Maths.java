package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Maths {
    public static double calculateSlope(Translation2d firstPoint, Translation2d secondPoint) {
        return (firstPoint.getY() - secondPoint.getY()) / (firstPoint.getX() - secondPoint.getX());
    }

    public static double getRelativeAngleFromTranslation(Pose2d currentPose, Translation2d targetTranslation) {
        final double poseDegrees = currentPose.getRotation().getDegrees();
        final double
                firstX = currentPose.getX(),
                secondX = targetTranslation.getX(),
                firstY = currentPose.getY(),
                secondY = targetTranslation.getY();
        final double ySignum = Math.signum(firstY - secondY);

        if (firstY == secondY && firstX == secondX) {
            return 0;
        }

        if (firstY == secondY) {
            return firstX < secondX ? poseDegrees - 90 : poseDegrees - 270;
        }

        if (firstX == secondX) {
            return poseDegrees * ySignum - (poseDegrees > 180 ? 360 : 0);
        }

        final double
                slope = calculateSlope(currentPose.getTranslation(), targetTranslation),
                slopeAsDegrees = Math.toDegrees(Math.atan(slope)),
                toReturn = slopeAsDegrees * ySignum + poseDegrees;

        return toReturn - (toReturn > 180 ? 360 : 0);
    }

}
