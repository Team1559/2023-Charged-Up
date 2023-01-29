package frc.lib;

import static frc.lib.SwerveTrajectory.MAXIMUM_LINEAR_ACCELERATION;
import static frc.lib.SwerveTrajectory.MAXIMUM_LINEAR_VELOCITY;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.lib.SwerveTrajectory.Point;

public class SwerveTrajectoryGenerator {
    private static final double DISTANCE_BETWEEN_POINTS = 0.02;
    private static final double SMOOTH_TOLERANCE        = 0.001;
    private static final double SMOOTH_WEIGHT           = 0.99;
    private static final double VELOCITY_PROPORTION     = 5;
    private static final double VELOCITY_POWER          = 0.25;

    public static SwerveTrajectory calculateTrajectory(Pose2d... waypoints) {
        // Generate the coordinates of the path
        Pose2d[] path = injectAndSmooth(waypoints);

        double[] curvatures = new double[path.length];
        double[] velocities = new double[path.length];
        double[] distances = new double[path.length];
        double[] predictedVelocities = new double[path.length];
        calculateVelocities(path, curvatures, velocities, predictedVelocities,
                distances);

        double[] times = new double[path.length];
        double[] accelerations = new double[path.length];
        calculateAccelerations(predictedVelocities, distances, times,
                accelerations);

        Point[] points = constructPoints(path, curvatures, velocities,
                distances, predictedVelocities, times, accelerations);
        return new SwerveTrajectory(points);
    }

    private static Point[] constructPoints(Pose2d[] path, double[] curvatures,
            double[] velocities, double[] distances,
            double[] predictedVelocities, double[] times,
            double[] accelerations) {
        Point[] points = new Point[path.length];
        for (int i = 0; i < path.length; i++) {
            points[i] = new Point(path[i], curvatures[i], velocities[i],
                    predictedVelocities[i], distances[i], times[i],
                    accelerations[i]);
        }
        return points;
    }

    private static void calculateAccelerations(double[] predictedVelocities,
            double[] distances, double[] times, double[] accelerations) {
        double time = 0;
        for (int i = 1; i < times.length; i++) {
            double avgVelocity = 0.5
                    * (predictedVelocities[i] + predictedVelocities[i - 1]);
            double distance = distances[i] - distances[i - 1];
            time += distance / avgVelocity;
            times[i] = time;

            double velocityChange = predictedVelocities[i]
                    - predictedVelocities[i - 1];
            double timeChange = times[i] - times[i - 1];
            accelerations[i - 1] = velocityChange / timeChange;
        }
    }

    private static void calculateVelocities(Pose2d[] path, double[] curvatures,
            double[] velocities, double[] predictedVelocities,
            double[] distances) {
        // Compute the commanded velocities and cumulative distances
        for (int i = 1; i < path.length - 1; i++) {
            distances[i] = distances[i - 1] + path[i].minus(path[i - 1])
                                                     .getTranslation()
                                                     .getNorm();
            curvatures[i] = calculateCurvature(path[i - 1], path[i],
                    path[i + 1]);
            if (Double.isNaN(curvatures[i])) {
                // Turn around in place
                velocities[i] = 0;
            } else {
                velocities[i] = Math.min(
                        VELOCITY_PROPORTION
                                / Math.pow(curvatures[i], VELOCITY_POWER),
                        MAXIMUM_LINEAR_VELOCITY);
            }
        }
        velocities[0] = MAXIMUM_LINEAR_VELOCITY;
        distances[path.length - 1] = distances[path.length - 2]
                + path[path.length - 1].minus(path[path.length - 2])
                                       .getTranslation()
                                       .getNorm();
        // Apply deceleration where needed
        for (int i = path.length - 2; i >= 0; i--) {
            double newVelocity = Math.sqrt(velocities[i + 1] * velocities[i + 1]
                    + 2 * MAXIMUM_LINEAR_ACCELERATION
                            * (distances[i + 1] - distances[i]));
            if (!Double.isNaN(newVelocity) && newVelocity < velocities[i]) {
                velocities[i] = newVelocity;
            }
        }
        /*
         * To project the actual performance of the robot, the commanded
         * velocities are turned into predicted velocities
         */
        System.arraycopy(velocities, 0, predictedVelocities, 0, path.length);
        predictedVelocities[0] = 0;
        for (int i = 1; i < path.length - 1; i++) {
            double newVelocity = Math.sqrt(
                    predictedVelocities[i - 1] * predictedVelocities[i - 1]
                            + 2 * MAXIMUM_LINEAR_ACCELERATION
                                    * (distances[i] - distances[i - 1]));
            if (!Double.isNaN(newVelocity) && newVelocity < velocities[i]) {
                predictedVelocities[i] = newVelocity;
            }
        }
    }

    private static Pose2d[] injectAndSmooth(Pose2d[] waypoints) {
        double[][] points = interpolate(waypoints);
        double[][] smoothPoints = smooth(points);

        // Convert points back to Pose2d
        Pose2d[] newPoints = new Pose2d[points.length];
        for (int i = 0; i < newPoints.length; i++) {
            newPoints[i] = new Pose2d(smoothPoints[i][0], smoothPoints[i][1],
                    Rotation2d.fromDegrees(points[i][2]));
        }
        return newPoints;
    }

    private static double[][] interpolate(Pose2d[] waypoints) {
        // Calculate how many points to insert into each segment and their
        // initial transforms
        int[] intermediatePointCounts = new int[waypoints.length - 1];
        double[][] deltas = new double[waypoints.length - 1][3];
        int totalPointCount = 1;
        for (int i = 0; i < deltas.length; i++) {
            deltas[i][0] = waypoints[i + 1].getX() - waypoints[i].getX();
            deltas[i][1] = waypoints[i + 1].getY() - waypoints[i].getY();
            deltas[i][2] = waypoints[i + 1].getRotation()
                                           .getDegrees()
                    - waypoints[i].getRotation()
                                  .getDegrees();
            double deltaD = Math.hypot(deltas[i][0], deltas[i][1]);
            int points = (int) (deltaD / DISTANCE_BETWEEN_POINTS);
            deltas[i][0] /= points;
            deltas[i][1] /= points;
            deltas[i][2] /= points;
            intermediatePointCounts[i] = points - 1;
            totalPointCount += points;
        }
        // Insert points by adding the deltas
        // Use 2D double array for less object creation and destruction during
        // editing
        double[][] points = new double[totalPointCount][3];
        int pointIndex = 0;
        double x, y, r, dx, dy, dr;
        for (int i = 0; i < deltas.length; i++) {
            // Copy original point
            x = waypoints[i].getX();
            y = waypoints[i].getY();
            r = waypoints[i].getRotation()
                            .getDegrees();
            points[pointIndex][0] = x;
            points[pointIndex][1] = y;
            points[pointIndex][2] = r;
            pointIndex++;
            // Create intermediate points
            dx = deltas[i][0];
            dy = deltas[i][1];
            dr = deltas[i][2];
            for (int j = 1; j <= intermediatePointCounts[i]; j++) {
                points[pointIndex][0] = x + dx * j;
                points[pointIndex][1] = y + dy * j;
                points[pointIndex][2] = r + dr * j;
                pointIndex++;
            }
        }
        points[pointIndex][0] = waypoints[waypoints.length - 1].getX();
        points[pointIndex][1] = waypoints[waypoints.length - 1].getY();
        points[pointIndex][2] = waypoints[waypoints.length - 1].getRotation()
                                                               .getDegrees();
        return points;
    }

    private static double[][] smooth(double[][] oldPoints) {
        double[][] newPoints = new double[oldPoints.length][oldPoints[0].length];
        for (int i = 0; i < newPoints.length; i++) {
            System.arraycopy(oldPoints[i], 0, newPoints[i], 0,
                    oldPoints[i].length);
        }

        double smoothWeight = SMOOTH_WEIGHT;
        double tolerance = SMOOTH_TOLERANCE;
        double pathWeight = 1 - smoothWeight;
        double change = tolerance;
        while (change >= tolerance) {
            change = 0;
            for (int i = 1; i < newPoints.length - 1; i++) {
                for (int j = 0; j < newPoints[i].length; j++) {
                    double delta = (pathWeight
                            * (oldPoints[i][j] - newPoints[i][j]))
                            + smoothWeight
                                    * (newPoints[i - 1][j] + newPoints[i + 1][j]
                                            - 2 * newPoints[i][j]);
                    newPoints[i][j] += delta;
                    change += Math.abs(delta);
                }
            }
        }

        return newPoints;
    }

    private static double calculateCurvature(Pose2d previous, Pose2d current,
            Pose2d next) {
        double x1 = previous.getX();
        double x2 = current.getX();
        double x3 = next.getX();
        double y1 = previous.getY();
        double y2 = current.getY();
        double y3 = next.getY();

        double dx1 = x1 - x2;
        double dx2 = x2 - x3;
        double dy1 = y1 - y2;
        double dy2 = y2 - y3;
        double a = dx1 * dy2 - dy1 * dx2;
        double b = dx1 * dx2 + dy1 * dy2;

        if (Math.abs(a) > 1e-9) {
            // Standard case
            return Math.abs(a / (b * Math.hypot(b, a)));
        }

        double dx3 = x3 - x1;
        double dy3 = y3 - y1;

        // Points are collinear, check for direction change
        double d12 = Math.hypot(dx1, dy1);
        double d23 = Math.hypot(dx2, dy2);
        double d13 = Math.hypot(dx3, dy3);
        if (d12 + d23 - d13 < 1e-6) {
            // 2 is between 1 and 3
            return 0;
        } else {
            // 2 is beyond 1 or 3
            return Double.NaN;
        }
    }

    public static void main(String... args) {
        Rotation2d DEGREES_0 = new Rotation2d();
        Rotation2d DEGREES_180 = Rotation2d.fromDegrees(180);

        Pose2d startPose1 = new Pose2d(2.2, 4.6, DEGREES_180);
        Pose2d startPose2 = new Pose2d(2.2, 2.7, DEGREES_180);
        Pose2d startPose3 = new Pose2d(2.2, 0.75, DEGREES_180);

        Pose2d chargingDock = new Pose2d(3.87, 2.7, DEGREES_180);

        // Piece Position 1X = (6.70, 4.60, 0)
        Pose2d fieldPiece1 = new Pose2d(6.7, 4.6, DEGREES_0);
        // Piece Position 2A = (6.70, 3.40, 0)
        Pose2d fieldPiece2a = new Pose2d(6.7, 3.4, DEGREES_0);
        // Piece Position 2B = (6,70, 2.10, 0)
        Pose2d fieldPiece2b = new Pose2d(6.7, 2.1, DEGREES_0);
        // Piece Position 3X = (6.70, 0.90, 0)
        Pose2d fieldPiece3 = new Pose2d(6.7, 0.9, DEGREES_0);

        Pose2d[] waypoints = { startPose1, fieldPiece1 };
        SwerveTrajectory trajectory = SwerveTrajectoryGenerator.calculateTrajectory(
                waypoints);

        System.out.println(trajectory.toString("\n"));
    }
}
