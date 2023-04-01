// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.FeatureFlags.ARM_ENABLED;
import static frc.robot.Constants.FeatureFlags.CHASSIS_ENABLED;
import static frc.robot.Constants.FeatureFlags.GRABBER_ENABLED;
import static frc.robot.Constants.FeatureFlags.VISION_ENABLED;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.DTXboxController;

import frc.robot.commands.ScoreCommands;
import frc.robot.commands.SwerveHoldPositionCommand;
import frc.robot.commands.SwerveTeleopDriveCommand;
import frc.robot.commands.TeleopWristAngleCommand;
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
    private final AutoRouteChooser autoRouteChooser;
    private final AutoRoutes       autoRoutes;
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
            armWrist = new ArmWrist();
            base.setHigherSegment(elbow);
            elbow.setLowerSegment(base);
            elbow.setHigherSegment(armWrist);
            armWrist.setLowerSegment(elbow);
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

        autoRoutes = new AutoRoutes(swerve, arm, wrist, claw, vision);
        autoRouteChooser = new AutoRouteChooser(autoRoutes);

        configureBindings();

        Class c = AutoRoutes.class;
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
    private enum Piece {
        NOT_PRESSED,
        PRESSED
    }

    private Piece selectModifier() {
        return controller1.getLeftBumper() ? Piece.PRESSED : Piece.NOT_PRESSED;
    }

    private void configureBindings() {
        if (ARM_ENABLED) {
            controller1.yButton.onTrue(new SelectCommand(
                    Map.ofEntries(
                            Map.entry(Piece.NOT_PRESSED, ScoreCommands.moveToConeHigh(arm, wrist)),
                            Map.entry(Piece.PRESSED, ScoreCommands.moveToCubeHigh(arm, wrist))),
                    this::selectModifier));
            controller1.xButton.onTrue(new SelectCommand(
                    Map.ofEntries(
                            Map.entry(Piece.NOT_PRESSED, ScoreCommands.moveToConeMid(arm, wrist)),
                            Map.entry(Piece.PRESSED, ScoreCommands.moveToCubeMid(arm, wrist))),
                    this::selectModifier));
            controller1.bButton.onTrue(new SelectCommand(
                    Map.ofEntries(
                            Map.entry(Piece.NOT_PRESSED, ScoreCommands.moveToConeLow(arm, wrist)),
                            Map.entry(Piece.PRESSED, ScoreCommands.moveToCubeLow(arm, wrist))),
                    this::selectModifier));
            controller1.aButton.onTrue(
                    new SelectCommand(
                            Map.ofEntries(
                                    Map.entry(Piece.NOT_PRESSED,
                                            arm.moveToPosition(Arm.Position.TRAVEL)),
                                    Map.entry(Piece.PRESSED, ScoreCommands.moveToTravel(arm))),
                            this::selectModifier));
            controller1.startButton.onTrue(arm.armPanicCommand());
        }

        if (GRABBER_ENABLED) {
            Command teleopWristCommand = new TeleopWristAngleCommand(wrist, controller1);
            wrist.setDefaultCommand(teleopWristCommand);
            controller1.leftStickButton.onTrue(claw.closeClawCommand());
            controller1.rightStickButton.onTrue(claw.openClawCommand());
        }

        if (GRABBER_ENABLED && ARM_ENABLED) {
            controller1.rightBumper.onTrue(new SelectCommand(
                    Map.ofEntries(
                            Map.entry(Piece.NOT_PRESSED,
                                    ScoreCommands.pickupConeCommand(arm, claw)),
                            Map.entry(Piece.PRESSED, ScoreCommands.pickupCubeCommand(arm, claw))),
                    this::selectModifier));
        }

        if (CHASSIS_ENABLED) {
            swerve.setDefaultCommand(new SwerveTeleopDriveCommand(swerve, controller0));
            controller0.yButton.onTrue(new InstantCommand(swerve::initialize, swerve));
            controller0.aButton.and(controller0.bButton)
                               .whileTrue(new SwerveHoldPositionCommand(swerve));
        }

        CommandScheduler.getInstance()
                        .schedule(new RunCommand(() -> SmartDashboard.putBoolean("Cube modifier",
                                selectModifier() == Piece.PRESSED)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoRouteChooser.getSelectedCommand();
    }

    public void swerveInit() {
        if (CHASSIS_ENABLED) {
            swerve.initialize();
        }
    }
}
