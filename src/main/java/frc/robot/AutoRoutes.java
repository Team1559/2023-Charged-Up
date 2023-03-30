package frc.robot;

import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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

    // @format:off
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
    private static final Rotation2d DEGREES_195 = Rotation2d.fromDegrees(195);
    private static final Rotation2d DEGREES_210 = Rotation2d.fromDegrees(210);
    private static final Rotation2d DEGREES_225 = Rotation2d.fromDegrees(225);
    private static final Rotation2d DEGREES_240 = Rotation2d.fromDegrees(240);
    private static final Rotation2d DEGREES_255 = Rotation2d.fromDegrees(255);
    private static final Rotation2d DEGREES_270 = Rotation2d.fromDegrees(270);
    private static final Rotation2d DEGREES_285 = Rotation2d.fromDegrees(285);
    private static final Rotation2d DEGREES_300 = Rotation2d.fromDegrees(300);
    private static final Rotation2d DEGREES_315 = Rotation2d.fromDegrees(315);
    private static final Rotation2d DEGREES_330 = Rotation2d.fromDegrees(330);
    private static final Rotation2d DEGREES_345 = Rotation2d.fromDegrees(345);
    // @format:on

    // ----------------
    // Common Positions
    // ----------------

    // Game piece 4 is closest to substations, 1 is farthest
    // @format:off
    private static final Pose2d GAME_PIECE_1 = new Pose2d(6.79, 0.92, DEGREES_0);
    private static final Pose2d GAME_PIECE_2 = new Pose2d(6.79, 2.14, DEGREES_0);
    private static final Pose2d GAME_PIECE_3 = new Pose2d(6.79, 3.36, DEGREES_0);
    private static final Pose2d GAME_PIECE_4 = new Pose2d(6.90, 4.40, DEGREES_255);

    private static final Pose2d CHARGING_STATION_CENTER = new Pose2d(3.90, 2.75, DEGREES_0);
    // @format:on

    /**
     * Start positions
     * <p>
     * START_POINT_1 is aligned with cone scoring position closest to outside
     * wall by judges tables
     * <p>
     * START_POINT_2A is aligned with cone scoring position at Grid 2 closest to
     * outside wall by judges tables
     * <p>
     * START_POINT_2B is aligned with cone scoring position at Grid 2 closest to
     * inside wall of opponents loading area
     * <p>
     * START_POINT_3 is aligned with cone scoring position closest to inside
     * wall of opponents loading area
     */
    // @format:off
    private static final Pose2d START_POINT_1  = new Pose2d(1.81, 0.513, DEGREES_180);
    private static final Pose2d START_POINT_2A = new Pose2d(1.81, 2.18, DEGREES_180);
    private static final Pose2d START_POINT_2B = new Pose2d(1.81, 3.3, DEGREES_180);
    private static final Pose2d START_POINT_3  = new Pose2d(1.81, 4.983, DEGREES_180);
    // @format:on

    /**
     * Cube score positions
     * <p>
     * SCORE_POINT_CUBE_1 is cube scoring position next to START_POINT_1, but 2
     * inches back in x-axis
     * <p>
     * SCORE_POINT_CUBE_3 is cube scoring position next to START_POINT_3, but 2
     * inches back in x-axis
     */
    // @format:off
    private static final Pose2d SCORE_POINT_CUBE_1 = new Pose2d(1.86, 0.97, DEGREES_180);
    private static final Pose2d SCORE_POINT_CUBE_3 = new Pose2d(1.86, 4.42, DEGREES_180);
    // @format:on

    // -------------
    // Defined Paths
    // -------------

    // @format:off
    private static final Pose2d S1_EXIT_C     = new Pose2d(5.2, 0.8, DEGREES_180);
    private static final Pose2d S1_EXIT_POINT = new Pose2d(6.0, 0.919, DEGREES_180);
    // @format:on

    // Way point positions for Starting Position 1 to Game Piece 1 route
    // @format:off
    private static final Pose2d S1_P1_A = new Pose2d(2.1, 0.8, DEGREES_180);
    private static final Pose2d S1_P1_B = new Pose2d(4.7, 0.8, DEGREES_180);
    private static final Pose2d S1_P1_C = new Pose2d(5.2, 2.8, DEGREES_150);
    private static final Pose2d S1_P1_D = new Pose2d(6.0, 0.9, DEGREES_30);
    private static final Pose2d S1_P1_E = new Pose2d(6.3, 0.919, DEGREES_0);
    // @format:on

    // Start 1 Exit Community path
    // @format:off
    private static final Pose2d[] START_1_EXIT_COMMUNITY_PATH = {
        START_POINT_1,
        S1_P1_A,
        S1_P1_B,
        S1_EXIT_C,
        S1_EXIT_POINT
    };
    // @format:on

    // Start 1 to Game Piece 1 path
    // @format:off
    private static final Pose2d[] START_1_TO_GAME_PIECE_1_PATH = {
        START_POINT_1,
        S1_P1_A,
        S1_P1_B,
        S1_P1_C,
        S1_P1_D,
        S1_P1_E,
        GAME_PIECE_1
    };
    // @format:on

    // Way point positions for Starting Position 3 Exit Community
    // @format:off
    private static final Pose2d S3_COM_A = new Pose2d(2.26, 4.72, DEGREES_180);
    private static final Pose2d S3_COM_B = new Pose2d(4.25, 4.8, DEGREES_180);
    private static final Pose2d S3_COM_C = new Pose2d(6.9, 5.9, DEGREES_210);
    private static final Pose2d S3_COM_D = new Pose2d(7.5, 5.5, DEGREES_255);
    // @format:on

    // Start 3 Exit Community path
    // @format:off
    private static final Pose2d[] START_3_EXIT_COMMUNITY_PATH = {
        START_POINT_3,
        S3_COM_A,
        S3_COM_B,
        S3_COM_C,
        S3_COM_D
    };
    // @format:on

    // Start 3 to Game Piece 4 path
    // @format:off
    private static final Pose2d[] START_3_TO_GAME_PIECE_4_PATH = {
        START_POINT_3,
        S3_COM_A,
        S3_COM_B,
        S3_COM_C,
        S3_COM_D,
        GAME_PIECE_4
    };
    // @format:on

    // Game Piece 1 to Score Cube 1 path
    // @format:off
    private static final Pose2d[] GAME_PIECE_1_TO_SCORE_CUBE_1_PATH = {
        GAME_PIECE_1,
        S1_P1_E,
        S1_P1_D,
        S1_P1_C,
        S1_P1_B,
        S1_P1_A,
        SCORE_POINT_CUBE_1
    };
    // @format:on

    // Game Piece 4 to Score Cube 3 path
    // @format:off
    private static final Pose2d[] GAME_PIECE_4_TO_SCORE_CUBE_3_PATH = {
        GAME_PIECE_4,
        S3_COM_D,
        S3_COM_C,
        S3_COM_B,
        S3_COM_A,
        SCORE_POINT_CUBE_3
    };
    // @format:on

    // --------------------
    // End of defined paths
    // --------------------

    static {
        if (RobotBase.isSimulation()) {
            new Thread(AutoRoutes::simulateTrajectories).start();
        }
    }

    private final SwerveDrive  swerve;
    private final Arm          arm;
    private final GrabberWrist wrist;
    private final GrabberClaw  claw;
    private final Vision       vision;

    // Index 0 is blue, index 1 is red (mirrored)
    private final SwerveTrajectory[] leave1Traj;
    private final SwerveTrajectory[] leave1ToGamePiece1Traj;
    private final SwerveTrajectory[] leave3Traj;
    private final SwerveTrajectory[] leave3ToGamePiece4Traj;
    private final SwerveTrajectory[] gamePiece1ToScoreCube1Traj;
    private final SwerveTrajectory[] gamePiece4ToScoreCube3Traj;

    public AutoRoutes(SwerveDrive swerve, Arm arm, GrabberWrist wrist, GrabberClaw claw,
            Vision vision) {
        this.swerve = swerve;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.vision = vision;

        // Define all trajectories used by auto routes
        leave1Traj = new SwerveTrajectory[] { makeTrajectory(START_1_EXIT_COMMUNITY_PATH),
                makeTrajectory(mirror(START_1_EXIT_COMMUNITY_PATH)) };
        leave1ToGamePiece1Traj = new SwerveTrajectory[] {
                makeTrajectory(START_1_TO_GAME_PIECE_1_PATH),
                makeTrajectory(mirror(START_1_TO_GAME_PIECE_1_PATH)) };
        leave3Traj = new SwerveTrajectory[] { makeTrajectory(START_3_EXIT_COMMUNITY_PATH),
                makeTrajectory(mirror(START_3_EXIT_COMMUNITY_PATH)) };
        leave3ToGamePiece4Traj = new SwerveTrajectory[] {
                makeTrajectory(START_3_TO_GAME_PIECE_4_PATH),
                makeTrajectory(mirror(START_3_TO_GAME_PIECE_4_PATH)) };
        gamePiece1ToScoreCube1Traj = new SwerveTrajectory[] {
                makeTrajectory(GAME_PIECE_1_TO_SCORE_CUBE_1_PATH),
                makeTrajectory(mirror(GAME_PIECE_1_TO_SCORE_CUBE_1_PATH)) };
        gamePiece4ToScoreCube3Traj = new SwerveTrajectory[] {
                makeTrajectory(GAME_PIECE_4_TO_SCORE_CUBE_3_PATH),
                makeTrajectory(mirror(GAME_PIECE_4_TO_SCORE_CUBE_3_PATH)) };
    }

    // -------------------
    // Autonomous commands
    // -------------------

    public Command scoreConeStayCmd() {
        return scoreConeHigh().andThen(startArmToTravel());
    }

    public Command leave1Cmd() {
        return setStartPose(START_POINT_1).andThen(startArmToTravel())
                                          .andThen(leave1Traj());
    }

    public Command scoreConeLeave1Cmd() {
        return setStartPose(START_POINT_1).andThen(scoreConeHigh())
                                          .andThen(startArmToTravel())
                                          .andThen(leave1Traj());
    }

    public Command scoreConeLeave1PickupCube1Cmd() {
        return setStartPose(START_POINT_1).andThen(scoreConeHigh())
                                          .andThen(startArmToTravel())
                                          .andThen(leave1ToPiece1Traj())
                                          .andThen(pickupCube());
    }

    public Command scoreConeLeave1PickupCube1ScoreCmd() {
        return setStartPose(START_POINT_1).andThen(scoreConeHigh())
                                          .andThen(startArmToTravel())
                                          .andThen(leave1ToPiece1Traj())
                                          .andThen(pickupCube())
                                          .andThen(piece1ToCubeScore1Traj())
                                          .andThen(scoreCubeHigh());
    }

    public Command leave3Cmd() {
        return setStartPose(START_POINT_3).andThen(startArmToTravel())
                                          .andThen(leave3Traj());
    }

    public Command scoreConeLeave3Cmd() {
        return setStartPose(START_POINT_3).andThen(scoreConeHigh())
                                          .andThen(startArmToTravel())
                                          .andThen(leave3Traj());
    }

    public Command scoreConeLeave3PickupCube4Cmd() {
        return setStartPose(START_POINT_3).andThen(scoreConeHigh())
                                          .andThen(startArmToTravel())
                                          .andThen(leave3ToPiece4Traj())
                                          .andThen(pickupCube());
    }

    public Command scoreConeLeave3PickupCube4ScoreCmd() {
        return setStartPose(START_POINT_3).andThen(scoreConeHigh())
                                          .andThen(startArmToTravel())
                                          .andThen(leave3ToPiece4Traj())
                                          .andThen(pickupCube())
                                          .andThen(piece4ToCubeScore3Traj())
                                          .andThen(scoreCubeHigh());
    }

    // ----------------------------------
    // Commands that form larger commands
    // ----------------------------------

    private static Command wait(double timeSeconds) {
        return new WaitCommand(timeSeconds);
    }

    private static Command print(String msg) {
        return new PrintCommand(msg);
    }

    private Command followTrajectory(SwerveTrajectory trajectory) {
        return new SwerveTrajectoryCommand(swerve, trajectory, vision);
    }

    private Command setStartPose(Pose2d pose) {
        return print("setStartPose: " + pose).andThen(new InstantCommand(() -> {
            swerve.getPoseEstimator()
                  .addVisionMeasurement(pose, WPIUtilJNI.now() * 1e-6, VecBuilder.fill(0, 0, 0));
        }));
    }

    private Command scoreConeHigh() {
        return print("scoreConeHigh").andThen(ScoreCommands.scoreConeHigh(arm, wrist, claw));
    }

    public Command scoreCubeHigh() {
        return print("scoreCubeHigh").andThen(ScoreCommands.scoreCubeHigh(arm, wrist, claw));
    }

    private Command pickupCube() {
        return print("pickupCube").andThen(ScoreCommands.pickupCubeCommand(arm, claw));
    }

    private Command startArmToTravel() {
        return print("armToTravel").andThen(ScoreCommands.moveToTravel(arm))
                                   .withTimeout(0.25);
    }

    private Command leave1Traj() {
        return print("leave1").andThen(followTrajectory(leave1Traj[trajIndex()]));
    }

    private Command leave3Traj() {
        return print("leave3").andThen(followTrajectory(leave3Traj[trajIndex()]));
    }

    private Command leave1ToPiece1Traj() {
        return print("leave1ToGamePiece1").andThen(
                followTrajectory(leave1ToGamePiece1Traj[trajIndex()]));
    }

    private Command leave3ToPiece4Traj() {
        return print("leave3ToGamePiece4").andThen(
                followTrajectory(leave3ToGamePiece4Traj[trajIndex()]));
    }

    private Command piece1ToCubeScore1Traj() {
        return print("gamePiece1ToCubeScore1").andThen(
                followTrajectory(gamePiece1ToScoreCube1Traj[trajIndex()]));
    }

    private Command piece4ToCubeScore3Traj() {
        return print("gamePiece4ToCubeScore3").andThen(
                followTrajectory(gamePiece4ToScoreCube3Traj[trajIndex()]));
    }

    // ----------------
    // Helper Methods
    // ----------------

    private static int trajIndex() {
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

    private static Pose2d[] reverse(Pose2d[] path) {
        Pose2d[] reversed = new Pose2d[path.length];
        for (int i = 0; i < path.length; i++) {
            reversed[path.length - i - 1] = path[i];
        }
        return reversed;
    }

    private static void simulateTrajectories() {
        Pose2d[][] paths = { START_3_TO_GAME_PIECE_4_PATH };
        SwerveTrajectory[] blueTrajectories = Arrays.stream(paths)
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
        }
    }

    private static void simulateTrajectory(Field2d field, SwerveTrajectory trajectory) {
        field.getObject("trajectory")
             .setTrajectory(trajectory.toTrajectory());
        field.setRobotPose(trajectory.points[0].pose);
        sleep(1000);
        for (int i = 0; i < trajectory.length - 1; i++) {
            field.setRobotPose(trajectory.points[i].pose);
            sleep((int) ((trajectory.points[i + 1].time - trajectory.points[i].time) * 2000));
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
