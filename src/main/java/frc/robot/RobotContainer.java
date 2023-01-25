// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.DTXboxController;
import frc.robot.commands.AutoTrajectoryCommand;
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
    private final Vision           vision;

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        controller0 = new DTXboxController(0);
        controller1 = new DTXboxController(1);
        swerve = new SwerveDrive(controller0);

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
        // TODO: implement
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        swerve.zeroYaw();
        Pose2d startState = swerve.getEstimatedPose();
        Pose2d state1 = startState.plus(
                new Transform2d(new Translation2d(-1.5, 0), new Rotation2d()));
        Pose2d state2 = state1.plus(
                new Transform2d(new Translation2d(0, -1.5), new Rotation2d()));
        Pose2d state3 = state2.plus(
                new Transform2d(new Translation2d(1.5, 0), new Rotation2d()));
        Pose2d state4 = state3.plus(
                new Transform2d(new Translation2d(0, 1.5), new Rotation2d()));
        PIDController xController = new PIDController(1, 0, 0);
        PIDController yController = new PIDController(1, 0, 0);
        PIDController rController = new PIDController(0.1, 0, 0);
        return new AutoTrajectoryCommand(swerve.getKinematics(),
                swerve::setStates, swerve::setAngle, swerve::getEstimatedPose,
                xController, yController, rController, state1, state2, state3,
                state4).andThen(swerve::stopDriving, swerve)
                       .andThen(() -> SmartDashboard.putBoolean("Auto status",
                               true));
    }
}
