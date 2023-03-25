package frc.robot;

import java.util.Arrays;
import java.lang.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

    // ----------------
    // Defined Paths
    // ----------------

    // Define common positions
    private static final Pose2d GAME_PIECE_1 = new Pose2d(6.791, 0.919, DEGREES_0);
    private static final Pose2d GAME_PIECE_2 = new Pose2d(6.791, 2.138, DEGREES_0);
    private static final Pose2d GAME_PIECE_3 = new Pose2d(6.791, 3.358, DEGREES_0);
    private static final Pose2d GAME_PIECE_4 = new Pose2d(7.07, 4.33, DEGREES_0);

    // Define charge station points, counter-clockwise, starting from bottom
    // left of blue
    private static final Pose2d CS_EDGE_1 = new Pose2d(2.9, 1.5, DEGREES_0);
    private static final Pose2d CS_EDGE_2 = new Pose2d(4.8, 4.0, DEGREES_0);
    private static final Pose2d CS_EDGE_3 = new Pose2d(4.8, 1.5, DEGREES_0);
    private static final Pose2d CS_EDGE_4 = new Pose2d(2.9, 4.0, DEGREES_0);
    private static final Pose2d CS_CENTER = new Pose2d(3.9, 2.75, DEGREES_0);

    // Start positions
    // START_POINT_1 is aligned with cone scoring position closest to outside
    // wall by judges tables
    private static final Pose2d START_POINT_1 = new Pose2d(1.81, 0.513, DEGREES_180);
    // START_POINT_2A is aligned with cone scoring position at Station 2
    // closest to outside wall by judges tables
    private static final Pose2d START_POINT_2A = new Pose2d(1.81, 2.18, DEGREES_180);
    // START_POINT_2B is aligned with cone scoring position at Station 2
    // closest to inside wall of opponents loading area
    private static final Pose2d START_POINT_2B = new Pose2d(1.81, 3.3, DEGREES_180);
    // START_POINT_3 is aligned with cone scoring position closest to inside
    // wall of opponents loading area
    private static final Pose2d START_POINT_3 = new Pose2d(1.81, 4.983, DEGREES_180);

    // Score positions
    // SCORE_POINT_CUBE_1 is cube scoring position next to START_POINT_1,
    // but 2 inches back in x-axis
    private static final Pose2d SCORE_POINT_CUBE_1 = new Pose2d(1.86, 0.97, DEGREES_180);
    // Score positions
    // SCORE_POINT_CUBE_3 is cube scoring position next to START_POINT_3,
    // but 2 inches back in x-axis
    private static final Pose2d SCORE_POINT_CUBE_3 = new Pose2d(1.86, 4.52, DEGREES_180);

    // Way point positions for Starting Position 1 to Game Piece 1 route
    private static final Pose2d S1_P1_A = new Pose2d(2.1, 0.8, DEGREES_180);
    private static final Pose2d S1_P1_B = new Pose2d(4.7, 0.8, DEGREES_180);
    private static final Pose2d S1_P1_C = new Pose2d(5.2, 0.8, DEGREES_150);
    private static final Pose2d S1_P1_D = new Pose2d(6.0, 0.9, DEGREES_30);
    private static final Pose2d S1_P1_E = new Pose2d(6.3, 0.919, DEGREES_0);

    private static final Pose2d S1_EXIT_C     = new Pose2d(5.2, 0.8, DEGREES_180);
    private static final Pose2d S1_EXIT_POINT = new Pose2d(6.0, 0.919, DEGREES_180);

    // Way point positions for Starting Position 3 Exit Community
    private static final Pose2d S3_COM_A = new Pose2d(2.26, 4.72, DEGREES_180);
    private static final Pose2d S3_COM_B = new Pose2d(2.94, 4.72, DEGREES_180);
    private static final Pose2d S3_COM_C = new Pose2d(4.83, 4.72, DEGREES_180);
    private static final Pose2d S3_COM_D = new Pose2d(5.52, 4.72, DEGREES_0);

    // Way point positions for Starting Position 3 to Game Piece 4 route
    private static final Pose2d S3_P4_A = new Pose2d(0, 0, DEGREES_180);
    private static final Pose2d S3_P4_B = new Pose2d(2.94, 4.58, DEGREES_180);
    private static final Pose2d S3_P4_C = new Pose2d(4.83, 4.58, DEGREES_180);
    private static final Pose2d S3_P4_D = new Pose2d(6.5, 4.58, DEGREES_180);
    private static final Pose2d S3_P4_E = new Pose2d(6.82, 4.58, DEGREES_180);

    // ----------------
    // Defined Paths
    // ----------------

    // Start 1 Exit Community path
    // @format:off
    private static final Pose2d[] START_1_EXIT_COMMUNITY_PATH = { 
        START_POINT_1, 
        S1_P1_A, 
        S1_P1_B,
        S1_EXIT_C, 
        S1_EXIT_POINT 
    };

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

    // Start 3 Exit Community path
    // @format:off
    private static final Pose2d[] START_3_EXIT_COMMUNITY_PATH = {
        START_POINT_3,
        S3_COM_A,
        S3_COM_B,
        S3_COM_C,
        S3_COM_D
    };

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

    // Game Piece 1 to Score Cube 1 path
    // @format:off
    private static final Pose2d[] GAME_PIECE_1_TO_SCORE_CUBE_1_PATH = { 
        GAME_PIECE_1, 
        S1_P1_E,
        S1_P1_D, 
        S1_P1_C, 
        S1_P1_B, 
        S1_P1_A, 
    };

    // Game Piece 4 to Score Cube 3 path
    // @format:off
    private static final Pose2d[] GAME_PIECE_4_TO_SCORE_CUBE_3_PATH = {
        GAME_PIECE_4,
        S3_COM_D,
        S3_COM_C,
        S3_COM_B,
        S3_COM_A,
        SCORE_POINT_CUBE_1 
    };

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
    //private final SwerveTrajectory[] piece1ToStart1Traj;
    //private final SwerveTrajectory[] start3ToPiece4Traj;
    //private final SwerveTrajectory[] piece4ToStart3Traj;
    //private final SwerveTrajectory[] start3ToScore3Traj;
    //private final SwerveTrajectory[] score3ToStart3Traj;
    //private final SwerveTrajectory[] start1ToScore1Traj;
    //private final SwerveTrajectory[] score1ToStart1Traj;

    public AutoRoutes(SwerveDrive swerve, Arm arm, GrabberWrist wrist, GrabberClaw claw,
            Vision vision) {
        this.swerve = swerve;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.vision = vision;

        // @format:off
        // Define all trajectories used by auto routes
        leave1Traj = new SwerveTrajectory[] { 
            makeTrajectory(START_1_EXIT_COMMUNITY_PATH),
            makeTrajectory(mirror(START_1_EXIT_COMMUNITY_PATH)) 
        };
        leave1ToGamePiece1Traj = new SwerveTrajectory[] {
            makeTrajectory(START_1_TO_GAME_PIECE_1_PATH),
            makeTrajectory(mirror(START_1_TO_GAME_PIECE_1_PATH)) 
        };
        leave3Traj = new SwerveTrajectory[] { 
            makeTrajectory(START_3_EXIT_COMMUNITY_PATH),
            makeTrajectory(mirror(START_3_EXIT_COMMUNITY_PATH)) 
        };
        leave3ToGamePiece4Traj = new SwerveTrajectory[] { 
            makeTrajectory(START_3_TO_GAME_PIECE_4_PATH),
            makeTrajectory(mirror(START_3_TO_GAME_PIECE_4_PATH)) 
        };
        gamePiece1ToScoreCube1Traj = new SwerveTrajectory[] {
            makeTrajectory(GAME_PIECE_1_TO_SCORE_CUBE_1_PATH),
            makeTrajectory(mirror(GAME_PIECE_1_TO_SCORE_CUBE_1_PATH)) 
        };
        gamePiece4ToScoreCube3Traj = new SwerveTrajectory[] {
            makeTrajectory(GAME_PIECE_4_TO_SCORE_CUBE_3_PATH),
            makeTrajectory(mirror(GAME_PIECE_4_TO_SCORE_CUBE_3_PATH)) 
        };
        //start1ToPiece1Traj = new SwerveTrajectory[] { makeTrajectory(START_1_TO_GAME_PIECE_1_PATH),
        //        makeTrajectory(mirror(START_1_TO_GAME_PIECE_1_PATH)) };
        //piece1ToStart1Traj = new SwerveTrajectory[] { makeTrajectory(PIECE_1_TO_START_1),
        //        makeTrajectory(mirror(PIECE_1_TO_START_1)) };
        //start3ToPiece4Traj = new SwerveTrajectory[] { makeTrajectory(START_3_TO_PIECE_4),
        //        makeTrajectory(mirror(START_3_TO_PIECE_4)) };
        //piece4ToStart3Traj = new SwerveTrajectory[] { makeTrajectory(PIECE_4_TO_START_3),
        //        makeTrajectory(mirror(PIECE_4_TO_START_3)) };
        //start3ToScore3Traj = new SwerveTrajectory[] { makeTrajectory(START_3_TO_SCORE_3),
        //        makeTrajectory(mirror(START_3_TO_SCORE_3)) };
        //score3ToStart3Traj = new SwerveTrajectory[] { makeTrajectory(SCORE_3_TO_START_3),
        //        makeTrajectory(mirror(SCORE_3_TO_START_3)) };
        //start1ToScore1Traj = new SwerveTrajectory[] { makeTrajectory(START_1_TO_SCORE_1),
        //        makeTrajectory(mirror(START_1_TO_SCORE_1)) };
        //score1ToStart1Traj = new SwerveTrajectory[] { makeTrajectory(SCORE_1_TO_START_1),
        //        makeTrajectory(mirror(SCORE_1_TO_START_1)) };
    }


    // @format:off
    // ----------------
    // Commands being used in auto route selector
    // ----------------
 
    public Command scoreConeStayCmd() {
        return arm.moveSequentially(Arm.Position.UPPER_CONE)
                  .andThen(new WaitCommand(1))
                  .andThen(claw.openClawCommand())
                  .andThen(arm.moveSequentially(Arm.Position.TRAVEL));
    }

    public Command leave1Cmd() {
        return new PrintCommand("leave1")
                .andThen(makeTrajectoryCommand(leave1Traj[trajectoryIndex()]));
    }

    public Command scoreLeave1Cmd() {
        return new InstantCommand(() -> setStartingPose(START_POINT_1))
                .andThen(scoreConeHighCmd())
                .andThen(armToTravelCmd().alongWith(leave1Cmd()));

    }

    public Command scoreLeave1ToGamePiece1Cmd() {
        return new InstantCommand(() -> setStartingPose(START_POINT_1))
                .andThen(scoreConeHighCmd())
                .andThen(armToTravelCmd().alongWith(leave1ToGamePiece1Cmd()))
                .andThen(pickupCubeCmd());
    }

    public Command scoreLeave1ToGamePiece1ScoreCubeCmd() {
        return new InstantCommand(() -> setStartingPose(START_POINT_1))
                .andThen(scoreConeHighCmd())
                .andThen(armToTravelCmd().alongWith(leave1ToGamePiece1Cmd()))
                .andThen(pickupCubeCmd())
                .andThen(gamePiece1ToCubeScore1Cmd());
    }

    public Command leave3Cmd() {
        return new PrintCommand("leave3").andThen(
                makeTrajectoryCommand(leave3Traj[trajectoryIndex()]));
    }

    public Command scoreLeave3Cmd() {
        return new InstantCommand(() -> setStartingPose(START_POINT_3))
                .andThen(scoreConeHighCmd())
                .andThen(armToTravelCmd().alongWith(leave3Cmd()));
    }

    public Command scoreLeave3ToGamePiece4Cmd() {
        return new InstantCommand(() -> setStartingPose(START_POINT_3))
                .andThen(scoreConeHighCmd())
                .andThen(armToTravelCmd().alongWith(leave3ToGamePiece4Cmd()))
                .andThen(pickupCubeCmd());
    }

    public Command scoreLeave3ToGamePiece4ScoreCubeCmd() {
        return new InstantCommand(() -> setStartingPose(START_POINT_3))
                .andThen(scoreConeHighCmd())
                .andThen(armToTravelCmd().alongWith(leave3ToGamePiece4Cmd()))
                .andThen(pickupCubeCmd())
                .andThen(gamePiece4ToCubeScore3Cmd());
    }

    // ----------------
    // Commands that are used by (make up) auto route commands
    // ----------------
    private Command makeTrajectoryCommand(SwerveTrajectory trajectory) {
        return new SwerveTrajectoryCommand(swerve, trajectory, vision);
    }

    public Command scoreConeHighCmd() {
        return new PrintCommand("scoreConeHigh")
                .andThen(ScoreCommands.scoreConeHigh(arm, wrist, claw));
    }

    public Command pickupCubeCmd() {
        return new PrintCommand("pickupCube")
                .andThen(ScoreCommands.pickupCubeCommand(arm, claw));
    }

    private Command armToTravelCmd() {
        return new PrintCommand("armToTravel")
                .andThen(ScoreCommands.moveToTravel(arm));
    }

    public Command leave1ToGamePiece1Cmd() {
        return new PrintCommand("leave1ToGamePiece1")
                .andThen(makeTrajectoryCommand(leave1ToGamePiece1Traj[trajectoryIndex()]));
    }

    public Command leave3ToGamePiece4Cmd() {
        return new PrintCommand("leave3ToGamePiece4")
                .andThen(makeTrajectoryCommand(leave3ToGamePiece4Traj[trajectoryIndex()]));
    }

    public Command gamePiece1ToCubeScore1Cmd() {
        return new PrintCommand("gamePiece1ToCubeScore1")
                .andThen(makeTrajectoryCommand(gamePiece1ToScoreCube1Traj[trajectoryIndex()]));
    }

    public Command gamePiece4ToCubeScore3Cmd() {
        return new PrintCommand("gamePiece4ToCubeScore3")
                .andThen(makeTrajectoryCommand(gamePiece4ToScoreCube3Traj[trajectoryIndex()]));
    }


    // ----------------
    // Helper Metods 
    // ----------------
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

    private static Pose2d[] reverse(Pose2d[] path) {
        Pose2d[] reversed = new Pose2d[path.length];
        for (int i = 0; i < path.length; i++) {
            reversed[path.length - i - 1] = path[i];
        }
        return reversed;
    }

    private static void simulateTrajectories() {
        Pose2d[][] paths = { GAME_PIECE_1_TO_SCORE_CUBE_1_PATH };
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

    private void setStartingPose(Pose2d startPose) {
        swerve.getPoseEstimator()
              .addVisionMeasurement(startPose, System.nanoTime() * Math.pow(10.0, -6),
                      VecBuilder.fill(0, 0, 0));
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


    //---------------
    // ANYTHING BELOW THIS IS NOT NEEDED AT THIS TIME
    // BUT MAY BE NEEDED AS MORE AUTO ROUTES ARE ADDED
    //---------------

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
    
    // Start points are in front of CONE nodes (-x, -x, +x)
    private static final Pose2d START_POINT_2 = new Pose2d(1.81, 2.189, DEGREES_180);

    private static final Pose2d SCORE_POINT_1 = new Pose2d(1.75, 1.631, DEGREES_180);
    private static final Pose2d SCORE_POINT_2 = new Pose2d(1.75, 2.189, DEGREES_180);
    private static final Pose2d SCORE_POINT_3 = new Pose2d(1.75, 3.84, DEGREES_180);

    private static final Pose2d START_TO_SCORE_1 = new Pose2d(START_POINT_1.getX(),
            SCORE_POINT_1.getY(), DEGREES_180);
    private static final Pose2d START_TO_SCORE_2 = new Pose2d(START_POINT_2.getX(),
            SCORE_POINT_2.getY(), DEGREES_180);
    private static final Pose2d START_TO_SCORE_3 = new Pose2d(START_POINT_3.getX(),
            SCORE_POINT_3.getY(), DEGREES_180);

    // private static final Pose2d GAME_PIECE_4 = new Pose2d(6.791, 4.577,
    // DEGREES_180);
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
    /*
     * Define specific routes Eventually maybe add mid points to have not a
     * sharp rotation We do not want to have each one rotating
     */

    // Temporarily turn off the formatter so we can have nice route lists.
    // @format:off

    private static final Pose2d[] START_3_TO_SCORE_3 = {
        START_POINT_3, START_TO_SCORE_3, SCORE_POINT_3
    };

    private static final Pose2d[] SCORE_3_TO_START_3 = reverse(START_3_TO_SCORE_3);

    private static final Pose2d[] START_1_TO_SCORE_1 = {
        START_POINT_1, START_TO_SCORE_1, SCORE_POINT_1
    };

    private static final Pose2d[] SCORE_1_TO_START_1 = reverse(START_1_TO_SCORE_1);


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

    // Piece 4 Start 3 Drive back to start
    private static final Pose2d[] PIECE_4_TO_START_3 = { GAME_PIECE_4, S3_P4_E, S3_P4_D, S3_P4_C,
            S3_P4_B, S3_P4_A, START_POINT_3 };
    // @format:on

    private static final Pose2d[] START_2_OVER_CHARGE_STATION = { S2_OCS_A, S2_OCS_B, S2_OCS_C,
            S2_OCS_D, S2_OCS_E };

}

// I am pretty sure that little if any of this is being
// Do we use?
// public Command start3Score3() {
// return makeTrajectoryCommand(start3ToScore3Traj[trajectoryIndex()]);
// }

// public Command score3Start3() {
// return makeTrajectoryCommand(score3ToStart3Traj[trajectoryIndex()]);
// }

// public Command start1Score1() {
// return makeTrajectoryCommand(start1ToScore1Traj[trajectoryIndex()]);
// }

// public Command score1Start1() {
// return makeTrajectoryCommand(score1ToStart1Traj[trajectoryIndex()]);
// }

// // public Command start1Piece1() {
// // return makeTrajectoryCommand(start1ToPiece1Traj[trajectoryIndex()]);
// // }

// public Command piece1Start1() {
// return makeTrajectoryCommand(piece1ToStart1Traj[trajectoryIndex()]);
// }

// public Command start3Piece4() {
// return makeTrajectoryCommand(start3ToPiece4Traj[trajectoryIndex()]);
// }

// public Command piece4Start3() {
// return makeTrajectoryCommand(piece4ToStart3Traj[trajectoryIndex()]);
// }

// public Command scoreCubeHigh() {
// return ScoreCommands.scoreCubeHigh(arm, wrist, claw);
// }

// public Command scoreCubeLeave1() {
// return scoreCubeHigh().andThen(leave1());
// }

// public Command scoreCubeLeave3() {
// return scoreCubeHigh().andThen(leave3());
// }

// public Command scoreCubePickupCubeReturn1() {
// return scoreCubeHigh().andThen(start1Piece1())
// .andThen(pickupCube())
// .andThen(piece1Start1());
// }

// public Command scoreConeLeave1() {
// return scoreConeHigh().andThen(leave1());
// }

// public Command scoreConeLeave3() {
// return scoreConeHigh().andThen(leave3());
// }

// public Command scoreConePickupConeReturn1() {
// return scoreConeHigh().andThen(start1Piece1())
// .andThen(pickupCone())
// .andThen(piece1Start1());
// }

// public Command scoreConePickupConeReturn3() {
// return scoreConeHigh().andThen(start3Piece4())
// .andThen(pickupCone())
// .andThen(piece4Start3());
// }

// private Command pickupCone() {
// return ScoreCommands.pickupConeCommand(arm, claw);
// }
// USED