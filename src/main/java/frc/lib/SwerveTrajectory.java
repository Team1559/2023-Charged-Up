package frc.lib;

import java.util.Arrays;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

import frc.robot.Constants;

public class SwerveTrajectory {
    public static final double MAXIMUM_LINEAR_VELOCITY      = 0.7
            * Constants.Swerve.MAXIMUM_LINEAR_VELOCITY;
    public static final double MAXIMUM_LINEAR_ACCELERATION  = MAXIMUM_LINEAR_VELOCITY;
    public static final double MAXIMUM_ANGULAR_VELOCITY     = 0.5
            * Constants.Swerve.MAXIMUM_ANGULAR_VELOCITY;
    public static final double MAXIMUM_ANGULAR_ACCELERATION = MAXIMUM_ANGULAR_VELOCITY;

    public static class Point {
        public final Pose2d pose;
        public final double curvature;
        public final double commandedVelocity;
        public final double predictedVelocity;
        public final double distance;

        public Point(Pose2d pose, double curvature, double commandedVelocity,
                double predictedVelocity, double distance) {
            this.pose = pose;
            this.curvature = curvature;
            this.commandedVelocity = commandedVelocity;
            this.predictedVelocity = predictedVelocity;
            this.distance = distance;
        }

        @Override
        public String toString() {
            return String.format(
                    "Point(X: %.2f, Y: %.2f, R: %.0f, C: %.0f, CV: %.1f, PV: %.1f, D: %.1f)",
                    pose.getX(), pose.getY(), pose.getRotation()
                                                  .getDegrees(),
                    curvature, commandedVelocity, predictedVelocity, distance);
        }
    }

    public final Point[] points;
    public final int     length;
    public final double  time;

    public SwerveTrajectory(Point... points) {
        this.points = points;
        length = points.length;
        time = calculateTime();
    }

    private double calculateTime() {
        double time = 0;
        for (int i = 1; i < points.length; i++) {
            double avgVelocity = 0.5 * (points[i].predictedVelocity
                    + points[i - 1].predictedVelocity);
            double distance = points[i].distance - points[i - 1].distance;
            time += distance / avgVelocity;
        }
        return time;
    }

    public Trajectory toTrajectory() {
        return new Trajectory(Arrays.stream(points)
                                    .map(p -> new Trajectory.State(0, 0, 0,
                                            p.pose, 0))
                                    .collect(Collectors.toList()));
    }

    @Override
    public String toString() {
        return toString("");
    }

    public String toString(String delimiter) {
        StringBuilder builder = new StringBuilder();
        builder.append("SwerveTrajectory([")
               .append(delimiter);
        int maxNumLen = (int) Math.ceil(Math.log10(points.length - 1));
        String formatString = String.format("%%%dd: ", maxNumLen);
        for (int i = 0; i < points.length; i++) {
            builder.append(String.format(formatString, i))
                   .append(points[i])
                   .append(delimiter);
        }
        builder.append(String.format("], T: %.1f)", time));
        return builder.toString();
    }
}
