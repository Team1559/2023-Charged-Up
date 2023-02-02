package frc.robot;

import java.util.Arrays;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.SwerveTrajectory;
import frc.lib.SwerveTrajectoryGenerator;

public class AutoRoutes {
    public static final double FIELD_LENGTH = 16.54;
    public static final double FIELD_WIDTH  = 8.02;

    private static final Rotation2d DEGREES_0   = new Rotation2d();
    private static final Rotation2d DEGREES_30  = Rotation2d.fromDegrees(30);
    private static final Rotation2d DEGREES_45  = Rotation2d.fromDegrees(45);
    private static final Rotation2d DEGREES_60  = Rotation2d.fromDegrees(60);
    private static final Rotation2d DEGREES_90  = Rotation2d.fromDegrees(90);
    private static final Rotation2d DEGREES_120 = Rotation2d.fromDegrees(120);
    private static final Rotation2d DEGREES_135 = Rotation2d.fromDegrees(135);
    private static final Rotation2d DEGREES_150 = Rotation2d.fromDegrees(150);
    private static final Rotation2d DEGREES_180 = Rotation2d.fromDegrees(180);

    // Define common positions
    public static final Pose2d START_POINT_1 = new Pose2d(2.2, 4.7,
            DEGREES_180);
    public static final Pose2d START_POINT_2 = new Pose2d(2.2, 2.75,
            DEGREES_180);
    public static final Pose2d START_POINT_3 = new Pose2d(2.2, 0.75,
            DEGREES_180);

    public static final Pose2d GAME_PIECE_1 = new Pose2d(6.8, 4.6, DEGREES_0);
    public static final Pose2d GAME_PIECE_2 = new Pose2d(6.8, 3.35, DEGREES_0);
    public static final Pose2d GAME_PIECE_3 = new Pose2d(6.8, 2.15, DEGREES_0);
    public static final Pose2d GAME_PIECE_4 = new Pose2d(6.8, 0.9, DEGREES_0);

    // Define specific routes
    private static final Pose2d S1_P1_A = new Pose2d(4.0, 4.7, DEGREES_180);
    private static final Pose2d S1_P1_B = new Pose2d(4.6, 4.8, DEGREES_150);
    private static final Pose2d S1_P1_C = new Pose2d(5.1, 4.9, DEGREES_90);
    private static final Pose2d S1_P1_D = new Pose2d(5.6, 4.7, DEGREES_30);
    private static final Pose2d S1_P1_E = new Pose2d(6.2, 4.6, DEGREES_0);

    public static final Pose2d[] START_1_TO_PIECE_1 = { START_POINT_1, S1_P1_A,
            S1_P1_B, S1_P1_C, S1_P1_D, S1_P1_E, GAME_PIECE_1 };
    public static final Pose2d[] PIECE_1_TO_START_1 = { GAME_PIECE_1, S1_P1_E,
            S1_P1_D, S1_P1_C, S1_P1_B, S1_P1_A, START_POINT_1 };

    private static final Pose2d S3_P4_A = new Pose2d(5.2, 0.75, DEGREES_180);
    private static final Pose2d S3_P4_B = new Pose2d(5.4, 0.85, DEGREES_150);
    private static final Pose2d S3_P4_C = new Pose2d(5.7, 0.9, DEGREES_90);
    private static final Pose2d S3_P4_D = new Pose2d(6.0, 0.9, DEGREES_30);
    private static final Pose2d S3_P4_E = new Pose2d(6.3, 0.9, DEGREES_0);

    public static final Pose2d[] START_3_TO_PIECE_4 = { START_POINT_3, S3_P4_A,
            S3_P4_B, S3_P4_C, S3_P4_D, S3_P4_E, GAME_PIECE_4 };
    public static final Pose2d[] PIECE_4_TO_START_3 = { GAME_PIECE_4, S3_P4_E,
            S3_P4_D, S3_P4_C, S3_P4_B, S3_P4_A, START_POINT_3 };

    static {
        if (RobotBase.isSimulation()) {
            new Thread(AutoRoutes::simulateTrajectories).start();
        }
    }

    public static Pose2d[] mirror(Pose2d[] path) {
        Pose2d[] mirrored = new Pose2d[path.length];
        for (int i = 0; i < path.length; i++) {
            mirrored[i] = new Pose2d(FIELD_LENGTH - path[i].getX(),
                    path[i].getY(), path[i].getRotation()
                                           .unaryMinus());
        }
        return mirrored;
    }

    private static void simulateTrajectories() {
        Pose2d[][] paths = { START_1_TO_PIECE_1, PIECE_1_TO_START_1,
                START_3_TO_PIECE_4, PIECE_4_TO_START_3 };
        SwerveTrajectory[] blueTrajectories = Arrays.stream(paths)
                                                .map(SwerveTrajectoryGenerator::calculateTrajectory)
                                                .toArray(
                                                        SwerveTrajectory[]::new);
        SwerveTrajectory[] redTrajectories = Arrays.stream(paths)
                                                 .map(AutoRoutes::mirror)
                                                 .map(SwerveTrajectoryGenerator::calculateTrajectory)
                                                 .toArray(
                                                         SwerveTrajectory[]::new);
        Field2d field = new Field2d();
        SmartDashboard.putData(field);
        while (true) {
            for (SwerveTrajectory trajectory : blueTrajectories) {
                simulateTrajectory(field, trajectory);
            }
            for (SwerveTrajectory trajectory : redTrajectories) {
                simulateTrajectory(field, trajectory);
            }
        }
    }

    private static void simulateTrajectory(Field2d field,
            SwerveTrajectory trajectory) {
        field.getObject("trajectory")
             .setTrajectory(trajectory.toTrajectory());
        field.setRobotPose(trajectory.points[0].pose);
        sleep(1000);
        for (int i = 0; i < trajectory.length; i++) {
            field.setRobotPose(trajectory.points[i].pose);
            if (i < trajectory.length - 1) {
                sleep((int) ((trajectory.points[i + 1].time
                        - trajectory.points[i].time) * 1000));
            }
        }
        sleep(1000);
    }

    private static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
