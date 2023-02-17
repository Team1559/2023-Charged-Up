// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.FeatureFlags.ARM_ENABLED;
import static frc.robot.Constants.FeatureFlags.CHASSIS_ENABLED;
import static frc.robot.Constants.FeatureFlags.GRABBER_ENABLED;
import static frc.robot.Constants.FeatureFlags.VISION_ENABLED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.DTXboxController;
import frc.robot.commands.TeleopWristAngleCommand;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmElbow;
import frc.robot.subsystems.arm.ArmWrist;
import frc.robot.subsystems.arm.FullArmCommands;
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
    private final FullArmCommands  arm;
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
            elbow = new ArmElbow(base); // new ArmElbow();
            armWrist = null; // new ArmWrist();
            arm = null; // new FullArmCommands(base, elbow, armWrist);
        } else {
            base = null;
            elbow = null;
            armWrist = null;
            arm = null;
        }
        if (CHASSIS_ENABLED) {
            swerve = new SwerveDrive(controller0);
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
        /**
         * Delete these commands after initial testing- also, START ARM @ 60
         * DEGREES!!!!!!!
         * Lower arm from top to test angle FF/PID
         */
        if (ARM_ENABLED) {
            controller0.aButton.onTrue(Commands.parallel(elbow.setAngleCommandPos(9), base.setAngleCommandPos(9)));
            controller0.bButton.onTrue(elbow.setAngleCommandPos(8));
            controller0.yButton.onTrue(elbow.setAngleCommandPos(7));
            controller0.leftBumper.onTrue(base.setAngleCommandPos(8));
            controller0.xButton.onTrue(Commands.parallel(base.resetEncoderForTesting(90), elbow.resetEncoderForTesting(0)));
        }
        if (GRABBER_ENABLED) {
            Command teleopWristCommand = new TeleopWristAngleCommand(wrist,
                    controller1);
            wrist.setDefaultCommand(teleopWristCommand);

            /**
             * Delete these 3 commands later, these are only for testing We will
             * create sequence commands later
             */
            controller1.aButton.onTrue(claw.closeClawCommand());
            controller1.yButton.onTrue(claw.openClawCommand());
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // TODO: implement autonomous command
        return null;
    }
}
