package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.DTXboxController;
import frc.robot.Constants;
import static frc.robot.Constants.Swerve.*;

public class SwerveDrive extends SubsystemBase {
    private final DTXboxController         controller;
    private final SwerveModule[]           modules;
    private final SwerveDriveKinematics    kinematics;
    private final Pigeon2                  gyro;
    private final SwerveDrivePoseEstimator poseEstimator;

    public SwerveDrive(DTXboxController c) {
        setSubsystem("Swerve Drive");
        setName(getSubsystem());

        controller = c;
        modules = new SwerveModule[4];
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(i);
        }
        gyro = new Pigeon2(Constants.Wiring.PIGEON_IMU);
        kinematics = new SwerveDriveKinematics(
                new Translation2d(MODULE_X, MODULE_Y),
                new Translation2d(MODULE_X, -MODULE_Y),
                new Translation2d(-MODULE_X, MODULE_Y),
                new Translation2d(-MODULE_X, -MODULE_Y));
        // poseEstimator = new SwerveDrivePoseEstimator(kinematics, null, null,
        // getEstimatedPosition());
        poseEstimator = null;
    }

    public void driveTeleop(double vx, double vy, double vr) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vr,
                Rotation2d.fromDegrees(0));
        // double linearVelocity = Math.hypot(speeds.vxMetersPerSecond,
        // speeds.vyMetersPerSecond);
        // if (linearVelocity < MINIMUM_LINEAR_VELOCITY
        // && speeds.omegaRadiansPerSecond < MINIMUM_ANGULAR_VELOCITY) {
        // // stop drive motor, no change to angle
        // // return
        // }

        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < modules.length; i++) {
            // SwerveModuleState.optimize(newStates[i],
            // modules[i].getSteerAngle());
            modules[i].setState(newStates[i]);
            SmartDashboard.putNumber("Velocity " + i,
                    newStates[i].speedMetersPerSecond);
        }
    }

    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Pose2d getEstimatedPosition() {
        return null;
    }

    public void addVisionPosition(Pose2d position, long time) {

    }

    private void updatePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = modules[i].getCurrentPosition();
        }
        poseEstimator.update(getGyroAngle(), positions);
    }

    @Override
    public void periodic() {
        if (DriverStation.isTeleop()) {
            double vx = controller.getLeftStickYSquared()
                    * MAXIMUM_LINEAR_VELOCITY;
            double vy = -controller.getLeftStickXSquared()
                    * MAXIMUM_LINEAR_VELOCITY;
            double vr = controller.getRightStickXSquared()
                    * MAXIMUM_ANGULAR_VELOCITY;
            vx = Math.round(vx);
            vy = Math.round(vy);
            vr = Math.round(vr);
            // driveTeleop(vx, vy, vr);
        }

        // updatePositions();
    }

    public SwerveModule[] getModules() {
        return modules;
    }
}
