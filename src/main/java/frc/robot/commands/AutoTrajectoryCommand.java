package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoTrajectoryCommand extends CommandBase {
    private final Pose2d[]                      destinations;
    private final SwerveDriveKinematics         kinematics;
    private final Consumer<SwerveModuleState[]> velocitySetter;
    private final Consumer<Rotation2d>          steerAngleSetter;
    private final Supplier<Pose2d>              poseSupplier;
    private final PIDController                 xController;
    private final PIDController                 yController;
    private final PIDController                 rController;
    private int                                 index;
    private long                                lastResetTime;

    public AutoTrajectoryCommand(SwerveDriveKinematics kinematics,
            Consumer<SwerveModuleState[]> velocitySetter,
            Consumer<Rotation2d> steerAngleSetter,
            Supplier<Pose2d> poseSupplier, PIDController xController,
            PIDController yController, PIDController rController,
            Pose2d... destinations) {
        this.destinations = destinations;
        this.kinematics = kinematics;
        this.velocitySetter = velocitySetter;
        this.steerAngleSetter = steerAngleSetter;
        this.poseSupplier = poseSupplier;
        this.xController = xController;
        this.yController = yController;
        this.rController = rController;
        index = 0;
        lastResetTime = System.currentTimeMillis();

        rController.enableContinuousInput(-180, 180);
        xController.setSetpoint(destinations[0].getX());
        yController.setSetpoint(destinations[0].getY());
        rController.setSetpoint(destinations[0].getRotation()
                                               .getDegrees());
        SmartDashboard.putBoolean("Auto status", false);
    }

    private Transform2d calculateDelta() {
        return destinations[index].minus(poseSupplier.get());
    }

    private void checkDelta() {
        Transform2d delta = calculateDelta();
        if (delta.getTranslation()
                 .getNorm() < 0.02
                && Math.abs(delta.getRotation()
                                 .getDegrees()) < 5) {
            index++;
            xController.setSetpoint(destinations[index].getX());
            yController.setSetpoint(destinations[index].getY());
            rController.setSetpoint(destinations[index].getRotation()
                                                       .getDegrees());
            steerAngleSetter.accept(destinations[index].getTranslation()
                                                       .getAngle());
            lastResetTime = System.currentTimeMillis();
        }
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - lastResetTime < 200) {
            return;
        }
        checkDelta();
        Pose2d currentPose = poseSupplier.get();
        double vx = xController.calculate(currentPose.getX());
        double vy = yController.calculate(currentPose.getY());
        double vr = rController.calculate(currentPose.getRotation()
                                                     .getDegrees());
        ChassisSpeeds newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy,
                vr, currentPose.getRotation());
        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(
                newSpeeds);
        velocitySetter.accept(newStates);
    }

    @Override
    public boolean isFinished() {
        return index >= destinations.length;
    }
}
