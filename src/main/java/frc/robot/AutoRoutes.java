package frc.robot;

import java.util.Arrays;

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
    public static final Pose2d START_POINT_3 = new Pose2d(2.2, 4.7,
            DEGREES_180);
    public static final Pose2d START_POINT_2 = new Pose2d(2.2, 2.75,
            DEGREES_180);
    public static final Pose2d START_POINT_1 = new Pose2d(2.2, 0.75,
            DEGREES_180);

    public static final Pose2d GAME_PIECE_4 = new Pose2d(6.8, 4.6, DEGREES_0);
    public static final Pose2d GAME_PIECE_3 = new Pose2d(6.8, 3.35, DEGREES_0);
    public static final Pose2d GAME_PIECE_2 = new Pose2d(6.8, 2.15, DEGREES_0);
    public static final Pose2d GAME_PIECE_1 = new Pose2d(6.8, 0.9, DEGREES_0);

    // Define charge station points, counter-clockwise, starting from bottom
    // left of blue
    public static final Pose2d CS_EDGE_1 = new Pose2d(2.9, 1.5, DEGREES_0);
    public static final Pose2d CS_EDGE_2 = new Pose2d(4.8, 4.0, DEGREES_0);
    public static final Pose2d CS_EDGE_3 = new Pose2d(4.8, 1.5, DEGREES_0);
    public static final Pose2d CS_EDGE_4 = new Pose2d(2.9, 4.0, DEGREES_0);
    public static final Pose2d CS_CENTER = new Pose2d(3.9, 2.75, DEGREES_0);

    /*
     * Define specific routes Eventually maybe add mid points to have not a
     * sharp rotation We do not want to have each one rotating
     */

    // Start 1 Piece 1 Drive to piece
    private static final Pose2d S1_P1_A = new Pose2d(1.49, 1.07, DEGREES_0);
    private static final Pose2d S1_P1_B = new Pose2d(2.94, 0.92, DEGREES_0);
    private static final Pose2d S1_P1_C = new Pose2d(4.83, 0.92, DEGREES_0);
    private static final Pose2d S1_P1_D = new Pose2d(6.5, 0.92, DEGREES_180);
    private static final Pose2d S1_P1_E = new Pose2d(6.82, 0.92, DEGREES_180);

    public static final Pose2d[] START_1_TO_PIECE_1 = { S1_P1_A, S1_P1_B,
            S1_P1_C, S1_P1_D, S1_P1_E };

    // Piece 1 Start 1 Drive back to start
    private static final Pose2d P1_S1_A = new Pose2d(6.82, 0.92, DEGREES_180);
    private static final Pose2d P1_S1_B = new Pose2d(4.83, 0.92, DEGREES_0);
    private static final Pose2d P1_S1_C = new Pose2d(2.94, 0.92, DEGREES_0);
    private static final Pose2d P1_S1_D = new Pose2d(1.49, 0.92, DEGREES_0);

        public static final Pose2d[] PIECE_1_TO_START_1 = { P1_S1_A, P1_S1_B,
                        P1_S1_C, P1_S1_D };

    // Position 1 Exit Community
    private static final Pose2d S1_COM_A = new Pose2d(1.49, 1.07, DEGREES_0);
    private static final Pose2d S1_COM_B = new Pose2d(2.94, 0.92, DEGREES_0);
    private static final Pose2d S1_COM_C = new Pose2d(4.83, 0.92, DEGREES_0);
    private static final Pose2d S1_COM_D = new Pose2d(5.52, 0.92, DEGREES_180);

        public static final Pose2d[] START_1_EXIT_COMMUNITY = { S1_COM_A,
                        S1_COM_B, S1_COM_C, S1_COM_D };

    // Start 3 Piece 4 Drive to piece
    private static final Pose2d S3_P4_A = new Pose2d(1.49, 4.42, DEGREES_0);
    private static final Pose2d S3_P4_B = new Pose2d(2.94, 4.58, DEGREES_0);
    private static final Pose2d S3_P4_C = new Pose2d(4.83, 4.58, DEGREES_0);
    private static final Pose2d S3_P4_D = new Pose2d(6.5, 4.58, DEGREES_180);
    private static final Pose2d S3_P4_E = new Pose2d(6.82, 4.58, DEGREES_180);

    public static final Pose2d[] START_3_TO_PIECE_4 = { S3_P4_A, S3_P4_B,
            S3_P4_C, S3_P4_D, S3_P4_E };

    // Piece 4 Start 3 Drive back to start
    private static final Pose2d P4_S3_A = new Pose2d(6.82, 4.58, DEGREES_180);
    private static final Pose2d P4_S3_B = new Pose2d(4.83, 4.58, DEGREES_0);
    private static final Pose2d P4_S3_C = new Pose2d(2.94, 4.58, DEGREES_0);
    private static final Pose2d P4_S3_D = new Pose2d(1.49, 4.42, DEGREES_0);

    public static final Pose2d[] PIECE_4_TO_START_3 = { P4_S3_A, P4_S3_B,
            P4_S3_C, P4_S3_D };

    // Position 3 Exit Community
    private static final Pose2d S3_COM_A = new Pose2d(1.49, 4.42, DEGREES_0);
    private static final Pose2d S3_COM_B = new Pose2d(2.94, 4.58, DEGREES_0);
    private static final Pose2d S3_COM_C = new Pose2d(4.83, 4.58, DEGREES_0);
    private static final Pose2d S3_COM_D = new Pose2d(5.52, 4.58, DEGREES_180);

    public static final Pose2d[] START_3_EXIT_COMMUNITY = { S3_COM_A, S3_COM_B,
            S3_COM_C, S3_COM_D };

    // Center of Charge Station From Start 2
    private static final Pose2d S2_CS_A = new Pose2d(1.49, 2.75, DEGREES_0);
    private static final Pose2d S2_CS_B = new Pose2d(2.94, 2.75, DEGREES_0);
    private static final Pose2d S2_CS_C = new Pose2d(3.88, 2.75, DEGREES_0);

    public static final Pose2d[] START_2_TO_CHARGE_STATION = { S2_CS_A, S2_CS_B,
            S2_CS_C };

    // Go over charge station from start 2
    private static final Pose2d S2_OCS_A = new Pose2d(1.49, 2.75, DEGREES_0);
    private static final Pose2d S2_OCS_B = new Pose2d(2.94, 2.75, DEGREES_0);
    private static final Pose2d S2_OCS_C = new Pose2d(4.19, 2.75, DEGREES_0);
    private static final Pose2d S2_OCS_D = new Pose2d(5.52, 2.75, DEGREES_0);
    private static final Pose2d S2_OCS_E = new Pose2d(5.52, 2.75, DEGREES_180);

    public static final Pose2d[] START_2_OVER_CHARGE_STATION = { S2_OCS_A,
            S2_OCS_B, S2_OCS_C, S2_OCS_D, S2_OCS_E };

    static {
        if (RobotBase.isSimulation()) {
            new Thread(AutoRoutes::simulateTrajectories).start();
        }
    }

    private AutoRoutes() {}

    public static Pose2d[] mirror(Pose2d[] path) {
        Pose2d[] mirrored = new Pose2d[path.length];
        for (int i = 0; i < path.length; i++) {
            mirrored[i] = new Pose2d(FIELD_LENGTH - path[i].getX(),
                    path[i].getY(),
                    Rotation2d.fromDegrees(180 - path[i].getRotation()
                                                        .getDegrees()));
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
