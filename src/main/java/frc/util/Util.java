package frc.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class Util {

    private Util() {}

    public static boolean epsilonEquals(double a, double b, double epsilon) {
      return (a - epsilon <= b) && (a + epsilon >= b);
    }
  
    public static boolean epsilonZero(double a, double epsilon) {
      return epsilonEquals(a, 0, epsilon);
    }

    public static double normalizeDegrees(double degrees) {
        degrees %= 360;
        degrees += 360;
        return degrees % 360;
      }

    public static double relativeAngularDifference(double currentAngle, double newAngle) {
        double normCurrentAngle = normalizeDegrees(currentAngle);
        double normNewAngle = normalizeDegrees(newAngle);
        double difference1 = Math.abs(normCurrentAngle - normNewAngle);
        double difference2 = Math.abs(360 - difference1);
        double difference = difference1 < difference2 ? difference1 : difference2;
    
        if ((normCurrentAngle + difference) % 360 == normNewAngle) return difference * -1;
        return difference;
      }
    

    public static double relativeAngularDifference(Rotation2d currentAngle, double newAngle) {
        return relativeAngularDifference(currentAngle.getDegrees(), newAngle);
      }
    
    public static double vectorToAngle(double x, double y) {
        double angle = Math.atan2(y, x);
        return (angle * (180 / Math.PI) + 360) % 360;
      }

    public static double angleSnap(double angle, double[] snaps) {
        double closest = snaps[0];
        for (double snap : snaps) {
        if (Math.abs(relativeAngularDifference(angle, snap))
            < Math.abs(relativeAngularDifference(angle, closest))) {
            closest = snap;
        }
        }
        return closest;
    }

    public static double vectorMagnitude(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
      }
}