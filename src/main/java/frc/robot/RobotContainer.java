// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.FeatureFlags.ARM_ENABLED;
import static frc.robot.Constants.FeatureFlags.CHASSIS_ENABLED;
import static frc.robot.Constants.FeatureFlags.GRABBER_ENABLED;
import static frc.robot.Constants.FeatureFlags.VISION_ENABLED;
import static frc.robot.Constants.Wiring.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.DTXboxController;
import frc.lib.SwerveTrajectory;
import frc.lib.SwerveTrajectoryGenerator;
import frc.robot.commands.SwerveTeleopDriveCommand;
import frc.robot.commands.SwerveTeleopSnapRotateCommand;
import frc.robot.commands.SwerveTrajectoryCommand;
import frc.robot.commands.TeleopWristAngleCommand;
import frc.robot.subsystems.Color;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmElbow;
import frc.robot.subsystems.arm.ArmWrist;
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
    private final Arm              arm;
    private final ArmBase          base;
    private final ArmElbow         elbow;
    private final ArmWrist         armWrist;
    private final Lighting         lighting;

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        controller0 = new DTXboxController(0);
        controller1 = new DTXboxController(1);
        if (ARM_ENABLED) {
            base = new ArmBase();
            elbow = new ArmElbow();
            base.setHigherSegment(elbow);
            elbow.setLowerSegment(base);
            armWrist = null;
            arm = new Arm(base, elbow, armWrist);
        } else {
            base = null;
            elbow = null;
            armWrist = null;
            arm = null;
        }
        if (CHASSIS_ENABLED) {
            swerve = new SwerveDrive();
        } else {
            swerve = null;
        }

        if (GRABBER_ENABLED) {
            wrist = new GrabberWrist();
            claw = new GrabberClaw();
        } else {
            wrist = null;
            claw = null;
        }

        if (VISION_ENABLED) {
            if (CHASSIS_ENABLED) {
                vision = new Vision(swerve.getPoseEstimator());
            } else {
                vision = new Vision(null);
            }
        } else {
            vision = null;
        }

        lighting = new Lighting(PWM_RED_PORT, PWM_GREEN_PORT, PWM_BLUE_PORT);
        lighting.setColor(Color.purpleColor);
        configureBindings();
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
        if (ARM_ENABLED) {
            controller0.aButton.onTrue(arm.moveToLocations(0));
            controller0.bButton.onTrue(arm.moveToLocations(1));
            controller0.xButton.onTrue(arm.moveToLocations(2));
            controller0.yButton.onTrue(arm.moveToLocations(3));
            controller0.rightStickButton.onTrue(arm.moveToLocations(4));

            // controller0.aButton.onTrue(Commands.parallel(
            // elbow.setAngleCommandPos(9), base.setAngleCommandPos(9)));
            // controller0.bButton.onTrue(elbow.setAngleCommandPos(7));
            // controller0.yButton.onTrue(elbow.setAngleCommandPos(0));
            // controller0.leftBumper.onTrue(base.setAngleCommandPos(6));
            controller0.leftStickButton.onTrue(Commands.parallel(base.resetEncoderForTesting(90),
                    elbow.resetEncoderForTesting(90)));
        }
        if (GRABBER_ENABLED) {
            Command teleopWristCommand = new TeleopWristAngleCommand(wrist, controller1);
            wrist.setDefaultCommand(teleopWristCommand);
            controller1.aButton.onTrue(claw.closeClawCommand());
            controller1.yButton.onTrue(claw.openClawCommand());
        }
        if (CHASSIS_ENABLED) {
            swerve.setDefaultCommand(new SwerveTeleopDriveCommand(swerve, controller0));
            controller0.leftBumper.onTrue(new SwerveTeleopSnapRotateCommand(swerve, false));
            controller0.rightBumper.onTrue(new SwerveTeleopSnapRotateCommand(swerve, true));
        }
        Command toGreen = new InstantCommand(() -> lighting.setColor(Color.greenColor));
        Command toRed = new InstantCommand(() -> lighting.setColor(Color.redColor));
        Command toBlue = new InstantCommand(() -> lighting.setColor(Color.blueColor));
        // Command toPurple = new InstantCommand(() ->
        // lighting.setColor(Color.purpleColor));
        Command toOff = new InstantCommand(() -> lighting.setColor(Color.off));
        controller0.aButton.onTrue(toGreen);
        controller0.bButton.onTrue(toRed);
        controller0.xButton.onTrue(toBlue);
        controller0.yButton.onTrue(toOff);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Rotation2d degrees180 = Rotation2d.fromDegrees(180);
        Pose2d[] waypoints = { new Pose2d(13, 2.75, degrees180), new Pose2d(11, 2.75, degrees180),
                new Pose2d(11, 4.75, degrees180), new Pose2d(13, 4.75, degrees180),
                new Pose2d(13, 2.75, degrees180) };
        SwerveTrajectory trajectory = SwerveTrajectoryGenerator.calculateTrajectory(waypoints);
        SmartDashboard.putNumber("Trajectory time", trajectory.time);
        swerve.displayTrajectory(trajectory);
        return new InstantCommand(() -> SmartDashboard.putBoolean("Auto active",
                true)).andThen(new SwerveTrajectoryCommand(swerve, trajectory))
                      .andThen(() -> SmartDashboard.putBoolean("Auto active", false));
    }
}
