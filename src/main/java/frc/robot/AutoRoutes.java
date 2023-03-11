package frc.robot;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.SwerveTrajectory;
import frc.lib.SwerveTrajectoryGenerator;

import frc.robot.commands.ScoreCommands;
import frc.robot.commands.SwerveTrajectoryCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.grabber.GrabberClaw;
import frc.robot.subsystems.grabber.GrabberWrist;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoRoutes {
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH  = 8.02;

    private static final Rotation2d DEGREES_0   = new Rotation2d();
    private static final Rotation2d DEGREES_15  = Rotation2d.fromDegrees(15);
    private static final Rotation2d DEGREES_30  = Rotation2d.fromDegrees(30);
    private static final Rotation2d DEGREES_45  = Rotation2d.fromDegrees(45);
    private static final Rotation2d DEGREES_60  = Rotation2d.fromDegrees(60);
    private static final Rotation2d DEGREES_75  = Rotation2d.fromDegrees(75);
    private static final Rotation2d DEGREES_90  = Rotation2d.fromDegrees(90);
    private static final Rotation2d DEGREES_105 = Rotation2d.fromDegrees(105);
    private static final Rotation2d DEGREES_120 = Rotation2d.fromDegrees(120);
    private static final Rotation2d DEGREES_135 = Rotation2d.fromDegrees(135);
    private static final Rotation2d DEGREES_150 = Rotation2d.fromDegrees(150);
    private static final Rotation2d DEGREES_165 = Rotation2d.fromDegrees(165);
    private static final Rotation2d DEGREES_180 = Rotation2d.fromDegrees(180);

    // Define common positions
    // Start points are in front of CONE nodes (-x, -x, +x)
    private static final Pose2d START_POINT_1 = new Pose2d(1.81, 0.513, DEGREES_180);
    private static final Pose2d START_POINT_2 = new Pose2d(1.81, 2.189, DEGREES_180);
    private static final Pose2d START_POINT_3 = new Pose2d(1.81, 4.983, DEGREES_180);

    private static final Pose2d GAME_PIECE_4 = new Pose2d(6.791, 4.577, DEGREES_0);
    private static final Pose2d GAME_PIECE_3 = new Pose2d(6.791, 3.358, DEGREES_0);
    private static final Pose2d GAME_PIECE_2 = new Pose2d(6.791, 2.138, DEGREES_0);
    private static final Pose2d GAME_PIECE_1 = new Pose2d(6.791, 0.919, DEGREES_0);
    // Define charge station points, counter-clockwise, starting from bottom
    // left of blue
    private static final Pose2d CS_EDGE_1 = new Pose2d(2.9, 1.5, DEGREES_0);
    private static final Pose2d CS_EDGE_2 = new Pose2d(4.8, 4.0, DEGREES_0);
    private static final Pose2d CS_EDGE_3 = new Pose2d(4.8, 1.5, DEGREES_0);
    private static final Pose2d CS_EDGE_4 = new Pose2d(2.9, 4.0, DEGREES_0);
    private static final Pose2d CS_CENTER = new Pose2d(3.9, 2.75, DEGREES_0);

    /*
     * Define specific routes Eventually maybe add mid points to have not a
     * sharp rotation We do not want to have each one rotating
     */
    private static final Pose2d S1_P1_A = new Pose2d(2.1, 0.8, DEGREES_180);
    private static final Pose2d S1_P1_B = new Pose2d(4.7, 0.8, DEGREES_180);
    private static final Pose2d S1_P1_C = new Pose2d(5.2, 0.8, DEGREES_150);
    private static final Pose2d S1_P1_D = new Pose2d(6.0, 0.9, DEGREES_30);
    private static final Pose2d S1_P1_E = new Pose2d(6.3, 0.919, DEGREES_0);
    // private static final Pose2d S1_P1_B = new Pose2d(2.94, 0.92, DEGREES_0);
    // private static final Pose2d S1_P1_C = new Pose2d(4.83, 0.92, DEGREES_0);
    // private static final Pose2d S1_P1_D = new Pose2d(6.5, 0.92, DEGREES_180);

    private static final Pose2d S1_EXIT_C     = new Pose2d(5.2, 0.8, DEGREES_180);
    private static final Pose2d S1_EXIT_POINT = new Pose2d(6.0, 0.919, DEGREES_180);

    // Temporarily turn off the formatter so we can have nice route lists.
    // @format:off

    // Start 1 Piece 1 Drive to piece
    private static final Pose2d[] START_1_TO_PIECE_1 = {
        START_POINT_1,
        S1_P1_A,
        S1_P1_B,
        S1_P1_C,
        S1_P1_D,
        S1_P1_E,
        GAME_PIECE_1
    };

    // Piece 1 Start 1 Drive back to start
    private static final Pose2d[] PIECE_1_TO_START_1 = {
        GAME_PIECE_1,
        S1_P1_E,
        S1_P1_D,
        S1_P1_C,
        S1_P1_B,
        S1_P1_A,
        START_POINT_1
    };

    // Position 1 Exit Community
    private static final Pose2d[] START_1_EXIT_COMMUNITY = {
        START_POINT_1,
        S1_P1_A,
        S1_P1_B,
        S1_EXIT_C,
        S1_EXIT_POINT
    };

    // Position 3 Exit Community
    private static final Pose2d S3_COM_B = new Pose2d(2.94, 4.58, DEGREES_0);
    private static final Pose2d S3_COM_C = new Pose2d(4.83, 4.58, DEGREES_0);
    private static final Pose2d S3_COM_D = new Pose2d(5.52, 4.58, DEGREES_180);

    private static final Pose2d[] START_3_EXIT_COMMUNITY = {
        START_POINT_3,
        S3_COM_B,
        S3_COM_C,
        S3_COM_D
    };

    // @format:on

    // private static final Pose2d S3_P4_B = new Pose2d(2.94, 4.58, DEGREES_0);
    // private static final Pose2d S3_P4_C = new Pose2d(4.83, 4.58, DEGREES_0);
    // private static final Pose2d S3_P4_D = new Pose2d(6.5, 4.58, DEGREES_180);
    // private static final Pose2d S3_P4_E = new Pose2d(6.82, 4.58,
    // DEGREES_180);
    private static final Pose2d S3_P4_A = new Pose2d(0, 0, DEGREES_180);
    private static final Pose2d S3_P4_B = new Pose2d(2.94, 4.58, DEGREES_180);
    private static final Pose2d S3_P4_C = new Pose2d(4.83, 4.58, DEGREES_180);
    private static final Pose2d S3_P4_D = new Pose2d(6.5, 4.58, DEGREES_180);
    private static final Pose2d S3_P4_E = new Pose2d(6.82, 4.58, DEGREES_180);

    // Start 3 Piece 4 Drive to piece
    // @format:off
    private static final Pose2d[] START_3_TO_PIECE_4 = {
        START_POINT_3,
        S3_P4_A,
        S3_P4_B,
        S3_P4_C,
        S3_P4_D,
        S3_P4_E,
        GAME_PIECE_4
    };

    // Piece 4 Start 3 Drive back to start
    private static final Pose2d[] PIECE_4_TO_START_3 = {
        GAME_PIECE_4,
        S3_P4_E,
        S3_P4_D,
        S3_P4_C,
        S3_P4_B,
        S3_P4_A,
        START_POINT_3
    };
    // @format:on

    // Center of Charge Station From Start 2
    private static final Pose2d S2_CS_A = new Pose2d(1.49, 2.75, DEGREES_0);
    private static final Pose2d S2_CS_B = new Pose2d(2.94, 2.75, DEGREES_0);
    private static final Pose2d S2_CS_C = new Pose2d(3.88, 2.75, DEGREES_0);

    private static final Pose2d[] START_2_TO_CHARGE_STATION = { S2_CS_A, S2_CS_B, S2_CS_C };

    // Go over charge station from start 2
    private static final Pose2d S2_OCS_A = new Pose2d(1.49, 2.75, DEGREES_0);
    private static final Pose2d S2_OCS_B = new Pose2d(2.94, 2.75, DEGREES_0);
    private static final Pose2d S2_OCS_C = new Pose2d(4.19, 2.75, DEGREES_0);
    private static final Pose2d S2_OCS_D = new Pose2d(5.52, 2.75, DEGREES_0);
    private static final Pose2d S2_OCS_E = new Pose2d(5.52, 2.75, DEGREES_180);

    private static final Pose2d[] START_2_OVER_CHARGE_STATION = { S2_OCS_A, S2_OCS_B, S2_OCS_C,
            S2_OCS_D, S2_OCS_E };

    static {
        if (RobotBase.isSimulation()) {
            new Thread(AutoRoutes::simulateTrajectories).start();
        }
    }

    private final SwerveDrive  swerve;
    private final Arm          arm;
    private final GrabberWrist wrist;
    private final GrabberClaw  claw;

    // Index 0 is blue, index 1 is red (mirrored)
    private final SwerveTrajectory[] leave1Traj;
    private final SwerveTrajectory[] leave3Traj;
    private final SwerveTrajectory[] start1ToPiece1Traj;
    private final SwerveTrajectory[] piece1ToStart1Traj;
    private final SwerveTrajectory[] start3ToPiece4Traj;
    private final SwerveTrajectory[] piece4ToStart3Traj;

    public AutoRoutes(SwerveDrive swerve, Arm arm, GrabberWrist wrist, GrabberClaw claw) {
        this.swerve = swerve;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;

        leave1Traj = new SwerveTrajectory[] { makeTrajectory(START_1_EXIT_COMMUNITY),
                makeTrajectory(mirror(START_1_EXIT_COMMUNITY)) };
        leave3Traj = new SwerveTrajectory[] { makeTrajectory(START_3_EXIT_COMMUNITY),
                makeTrajectory(mirror(START_3_EXIT_COMMUNITY)) };
        start1ToPiece1Traj = new SwerveTrajectory[] { makeTrajectory(START_1_TO_PIECE_1),
                makeTrajectory(mirror(START_1_TO_PIECE_1)) };
        piece1ToStart1Traj = new SwerveTrajectory[] { makeTrajectory(PIECE_1_TO_START_1),
                makeTrajectory(mirror(PIECE_1_TO_START_1)) };
        start3ToPiece4Traj = new SwerveTrajectory[] { makeTrajectory(START_3_TO_PIECE_4),
                makeTrajectory(mirror(START_3_TO_PIECE_4)) };
        piece4ToStart3Traj = new SwerveTrajectory[] { makeTrajectory(PIECE_4_TO_START_3),
                makeTrajectory(mirror(PIECE_4_TO_START_3)) };
    }

    public Command leave1() {
        return makeTrajectoryCommand(leave1Traj[trajectoryIndex()]);
    }

    public Command leave3() {
        return makeTrajectoryCommand(leave3Traj[trajectoryIndex()]);
    }

    public Command start1Piece1() {
        return makeTrajectoryCommand(start1ToPiece1Traj[trajectoryIndex()]);
    }

    public Command piece1Start1() {
        return makeTrajectoryCommand(piece1ToStart1Traj[trajectoryIndex()]);
    }

    public Command start3Piece4() {
        return makeTrajectoryCommand(start3ToPiece4Traj[trajectoryIndex()]);
    }

    public Command piece4Start3() {
        return makeTrajectoryCommand(piece4ToStart3Traj[trajectoryIndex()]);
    }

    public Command scoreCubeHigh() {
        return ScoreCommands.scoreCubeHigh(arm, wrist, claw);
    }

    public Command scoreConeHigh() {
        return ScoreCommands.scoreConeHigh(arm, wrist, claw);
    }

    public Command pickupCube() {
        return ScoreCommands.pickupCubeCommand(arm, claw);
    }

    public Command scoreCubeLeave1() {
        return scoreCubeHigh().andThen(leave1());
    }

    public Command scoreCubeLeave3() {
        return scoreCubeHigh().andThen(leave3());
    }

    public Command scoreCubePickupCubeReturn1() {
        return scoreCubeHigh().andThen(start1Piece1())
                              .andThen(pickupCube())
                              .andThen(piece1Start1());
    }

    public Command scoreConeLeave1() {
        return scoreConeHigh().andThen(leave1());
    }

    public Command scoreConeLeave3() {
        return scoreConeHigh().andThen(leave3());
    }

    public Command scoreConePickupConeReturn1() {
        return scoreConeHigh().andThen(start1Piece1())
                              .andThen(pickupCone())
                              .andThen(piece1Start1());
    }

    public Command scoreConePickupConeReturn3() {
        return scoreConeHigh().andThen(start3Piece4())
                              .andThen(pickupCone())
                              .andThen(piece4Start3());
    }

    private Command pickupCone() {
        return ScoreCommands.pickupConeCommand(arm, claw);
    }

    private Command makeTrajectoryCommand(SwerveTrajectory trajectory) {
        return new SwerveTrajectoryCommand(swerve, trajectory);
    }

    private static int trajectoryIndex() {
        return DriverStation.getAlliance() == Alliance.Red ? 1 : 0;
    }

    private static SwerveTrajectory makeTrajectory(Pose2d[] poses) {
        return SwerveTrajectoryGenerator.calculateTrajectory(poses);
    }

    private static Pose2d[] mirror(Pose2d[] path) {
        Pose2d[] mirrored = new Pose2d[path.length];
        for (int i = 0; i < path.length; i++) {
            mirrored[i] = new Pose2d(FIELD_LENGTH - path[i].getX(), path[i].getY(),
                    Rotation2d.fromDegrees(180 - path[i].getRotation()
                                                        .getDegrees()));
        }
        return mirrored;
    }

    private static void simulateTrajectories() {
        Pose2d[][] paths = { START_3_TO_PIECE_4, PIECE_4_TO_START_3 };
        SwerveTrajectory[] blueTrajectories = Arrays.stream(paths)
                                                    .map(SwerveTrajectoryGenerator::calculateTrajectory)
                                                    .toArray(SwerveTrajectory[]::new);
        SwerveTrajectory[] redTrajectories = Arrays.stream(paths)
                                                   .map(AutoRoutes::mirror)
                                                   .map(SwerveTrajectoryGenerator::calculateTrajectory)
                                                   .toArray(SwerveTrajectory[]::new);
        Field2d field = new Field2d();
        field.getObject("temp")
             .setPose(new Pose2d());
        SmartDashboard.putData(field);
        while (true) {
            for (SwerveTrajectory trajectory : blueTrajectories) {
                simulateTrajectory(field, trajectory);
            }
            // for (SwerveTrajectory trajectory : redTrajectories) {
            // simulateTrajectory(field, trajectory);
            // }
        }
    }

    private static void simulateTrajectory(Field2d field, SwerveTrajectory trajectory) {
        field.getObject("trajectory")
             .setTrajectory(trajectory.toTrajectory());
        field.setRobotPose(trajectory.points[0].pose);
        sleep(1000);
        for (int i = 0; i < trajectory.length - 1; i++) {
            field.setRobotPose(trajectory.points[i].pose);
            sleep((int) ((trajectory.points[i + 1].time - trajectory.points[i].time) * 1000));
        }
        field.setRobotPose(trajectory.points[trajectory.length - 1].pose);
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
