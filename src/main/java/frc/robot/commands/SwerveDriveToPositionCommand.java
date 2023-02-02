// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveDriveToPositionCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final SwerveDrive chassis;
    private final Pose2d      target;
    private final PIDController xController;
    private final PIDController yController;
    private final double       DISTANCE_TOLERANCE = 0.03;
    private final double       ANGLE_TOLERANCE = 3.0;

    public SwerveDriveToPositionCommand(SwerveDrive chassis, Pose2d target) {
        this.chassis = chassis;
        this.target = target;
        this.xController = new PIDController(0.5, 0, 0);
        this.yController = new PIDController(0.5, 0, 0);
        this.rController = new PIDController(0.5, 0, 0);

        this.xController.setSetpoint(target.getX());
        this.yController.setSetpoint(target.getY());
        this.rController.setSetpoint(target.getRotation().getDegrees());
        addRequirements(chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d current = this.chassis.getCurrentPose();
        double xDrive = this.xController.calculate(current.getX());
        double yDrive = this.xController.calculate(current.getY());
        double rDrive = this.xController.calculate(current.getRotation().getDegrees());
        this.chassis.driveVelocity(xDrive, yDrive, rDrive);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.chassis.stopDriving();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Pose2d current = this.chassis.getCurrentPose();
        Transform2d delta = this.target.minus(current);
        return Math.abs(delta.getX()) < DISTANCE_TOLERANCE &&
        Math.abs(delta.getY()) < DISTANCE_TOLERANCE &&
        Math.abs(delta.getRotation().getDegrees()) < ANGLE_TOLERANCE;
    }
}
