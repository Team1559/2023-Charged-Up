package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.MAXIMUM_ANGULAR_VELOCITY;
import static frc.robot.Constants.Swerve.MAXIMUM_LINEAR_VELOCITY;
import static frc.robot.Constants.Swerve.MINIMUM_ANGULAR_VELOCITY;
import static frc.robot.Constants.Swerve.MINIMUM_LINEAR_VELOCITY;
import static frc.robot.Constants.Swerve.MODULE_X;
import static frc.robot.Constants.Swerve.MODULE_Y;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.DTXboxController;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
    private final DTXboxController         controller;
    private final SwerveModule[]           modules;
    private final SwerveModulePosition[]   modulePositions;
    private final SwerveDriveKinematics    kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Pigeon2                  gyro;

    /**
     * @param c
     *        The {@link DTXboxController} to read input from during teleop
     */
    public SwerveDrive(DTXboxController c) {
        setSubsystem("Swerve Drive");
        setName(getSubsystem());

        controller = c;
        modules = new SwerveModule[4];
        modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(i);
            modulePositions[i] = modules[i].getCurrentPosition();
        }
        gyro = new Pigeon2(Constants.Wiring.PIGEON_IMU);
        kinematics = new SwerveDriveKinematics(
                new Translation2d(MODULE_X, MODULE_Y),
                new Translation2d(MODULE_X, -MODULE_Y),
                new Translation2d(-MODULE_X, MODULE_Y),
                new Translation2d(-MODULE_X, -MODULE_Y));
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(),
                modulePositions, new Pose2d(0, 0, getGyroAngle()));

        zeroYaw();
    }

    public void setStates(SwerveModuleState... states) {
        double minCosine = 1;
        for (int i = 0; i < modules.length; i++) {
            states[i] = SwerveModuleState.optimize(states[i],
                    modules[i].getSteerAngle());
            double cosine = Math.cos(Units.degreesToRadians(
                    modules[i].getSteerAngle()
                              .getDegrees()
                            - states[i].angle.getDegrees()));
            if (cosine < minCosine) {
                minCosine = cosine;
            }
        }
        SmartDashboard.putNumber("Cosine", minCosine);
        for (int i = 0; i < modules.length; i++) {
            states[i].speedMetersPerSecond *= minCosine;
            modules[i].setState(states[i]);
        }
    }

    public void stopDriving() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].stopDriving();
        }
    }

    public void zeroYaw() {
        gyro.setYaw(0);
    }

    public void setAngle(Rotation2d angle) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setSteerAngle(angle);
        }
    }

    /**
     * Calculates and commands {@link SwerveModuleState SwerveModuleStates} from
     * a set of field-relative speeds
     *
     * @param vx
     *        The desired linear velocity in the x-axis in meters per second
     *        (forwards is positive)
     * @param vy
     *        The desired linear velocity in the y-axis in meters per second
     *        (left is positive)
     * @param vr
     *        The desired rotational velocity in radians per second (CCW
     *        positive)
     */
    public void driveVelocity(double vx, double vy, double vr) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vr,
                getGyroAngle());

        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(speeds);
        setStates(newStates);
    }

    /**
     * @return the current yaw reading of the {@link Pigeon2} IMU in the range
     *         (-π, π)
     */
    public Rotation2d getGyroAngle() {
        // WPILib uses CW+, CTRE uses CCW+; need to negate
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * @return the {@link SwerveDrivePoseEstimator} used to calculate odometry
     */
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Updates the internal {@link SwerveDrivePoseEstimator PoseEstimator} with
     * encoder readings from each module
     */
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
            double vr = -controller.getRightStickXSquared()
                    * MAXIMUM_ANGULAR_VELOCITY;
            if (Math.abs(vx) > MINIMUM_LINEAR_VELOCITY
                    || Math.abs(vy) > MINIMUM_LINEAR_VELOCITY
                    || Math.abs(vr) > MINIMUM_ANGULAR_VELOCITY) {
                driveVelocity(vx, vy, vr);
            } else {
                stopDriving();
            }
        }

        updatePositions();
        Pose2d currentPose = poseEstimator.getEstimatedPosition();

        SmartDashboard.putNumber("Pos X", currentPose.getX());
        SmartDashboard.putNumber("Pos Y", currentPose.getY());
        SmartDashboard.putNumber("Pos Rot", currentPose.getRotation()
                                                       .getDegrees());
    }

    /**
     * @return a reference to the internal array of {@link SwerveModule
     *         SwerveModules}
     */
    public SwerveModule[] getModules() {
        return modules;
    }
}
