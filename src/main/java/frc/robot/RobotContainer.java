// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.DTXboxController;
import frc.robot.commands.TeleopWristAngleCommand;
import frc.robot.subsystems.grabber.GrabberClaw;
import frc.robot.subsystems.grabber.GrabberWrist;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final DTXboxController controller0;
    private final DTXboxController controller1;
    private final SwerveDrive      swerve;
    private final GrabberWrist     wrist;
    private final GrabberClaw      claw;
    private final Vision           vision;

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        controller0 = new DTXboxController(0);
        controller1 = new DTXboxController(1);
        swerve = new SwerveDrive(controller0);
        wrist = new GrabberWrist();
        claw = new GrabberClaw();

        configureBindings();
        vision = new Vision(swerve.getPoseEstimator());
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
     * with an arbitrary predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s
     * subclasses for {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        Command teleopWristCommand = new TeleopWristAngleCommand(wrist,
                controller1);
        wrist.setDefaultCommand(teleopWristCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PIDController xController = new PIDController(1, 0, 0);
        PIDController yController = new PIDController(1, 0, 0);
        ProfiledPIDController rController = new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.Swerve.MAXIMUM_ANGULAR_VELOCITY, Math.PI));
        HolonomicDriveController controller = new HolonomicDriveController(
                xController, yController, rController);
        List<Trajectory.State> states = new ArrayList<>();
        states.add(new State(0, 0, 0, swerve.getPoseEstimator()
                                            .getEstimatedPosition(),
                0));
        states.add(new State(2, 0, 0, new Pose2d(
                new Translation2d(13.513558, 4.424426), new Rotation2d()), 0));
        return new SwerveControllerCommand(new Trajectory(states),
                swerve::getEstimatedPose, swerve.getKinematics(), controller,
                swerve::setStates, swerve, vision);
    }
}
