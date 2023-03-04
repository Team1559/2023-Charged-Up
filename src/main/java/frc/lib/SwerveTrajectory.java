package frc.lib;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveTrajectory {
    public static class Point {
        public final Pose2d pose;
        public final double curvature;
        public final double commandedVelocity;
        public final double predictedVelocity;
        public final double distance;
        public final double time;
        public final double acceleration;

        public Point(Pose2d pose, double curvature, double commandedVelocity,
                double predictedVelocity, double distance, double time, double acceleration) {
            this.pose = pose;
            this.curvature = curvature;
            this.commandedVelocity = commandedVelocity;
            this.predictedVelocity = predictedVelocity;
            this.distance = distance;
            this.time = time;
            this.acceleration = acceleration;
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
        time = points[length - 1].time;
    }

    public Trajectory toTrajectory() {
        List<Trajectory.State> states = new ArrayList<>();
        for (int i = 0; i < points.length; i++) {
            Point p = points[i];
            states.add(new Trajectory.State(p.time, p.predictedVelocity, p.acceleration, p.pose,
                    p.curvature));
        }
        return new Trajectory(states);
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
