package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.MAXIMUM_ANGULAR_VELOCITY;
import static frc.robot.Constants.Swerve.MAXIMUM_LINEAR_VELOCITY;
import static frc.robot.Constants.Swerve.MODULE_X;
import static frc.robot.Constants.Swerve.MODULE_Y;
import static frc.robot.Constants.Swerve.ROTATION_KP;
import static frc.robot.Constants.Swerve.ENCODER_STDDEV;
import static frc.robot.Constants.Wiring.CANIVORE_BUS_ID;
import static frc.robot.Constants.Wiring.PIGEON_IMU;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SwerveTrajectory;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule[]           modules;
    private final SwerveModulePosition[]   modulePositions;
    private final SwerveDriveKinematics    kinematics;
    private final PIDController            rController;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Pigeon2                  gyro;
    private final double[]                 gyroDataArray;
    private Field2d                        field2d;
    private double                         rPIDSetpoint;

    public SwerveDrive() {
        setSubsystem("Swerve Drive");
        setName(getSubsystem());

        // Hardware & zero positions
        modules = new SwerveModule[4];
        modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(i);
            modulePositions[i] = modules[i].getCurrentPosition();
        }
        gyro = new Pigeon2(PIGEON_IMU, CANIVORE_BUS_ID);

        // Control software
        kinematics = new SwerveDriveKinematics(new Translation2d(MODULE_X, MODULE_Y),
                new Translation2d(MODULE_X, -MODULE_Y), new Translation2d(-MODULE_X, MODULE_Y),
                new Translation2d(-MODULE_X, -MODULE_Y));
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), modulePositions,
                new Pose2d(0, 0, getGyroAngle()),
                VecBuilder.fill(ENCODER_STDDEV, ENCODER_STDDEV, ENCODER_STDDEV),
                VecBuilder.fill(2, 2, 2));
        rController = new PIDController(ROTATION_KP, 0, 0);
        rController.enableContinuousInput(-Math.PI, Math.PI);
        rController.setTolerance(0.01);
        rPIDSetpoint = Double.NaN;
        gyroDataArray = new double[3];

        field2d = new Field2d();
        SmartDashboard.putData(field2d);

        gyro.configFactoryDefault();
    }

    public void setStates(SwerveModuleState... states) {
        double minCosine = 1;
        for (int i = 0; i < modules.length; i++) {
            states[i] = SwerveModuleState.optimize(states[i], modules[i].getSteerAngle());
            double cosine = Math.cos(Units.degreesToRadians(modules[i].getSteerAngle()
                                                                      .getDegrees()
                    - states[i].angle.getDegrees()));
            if (cosine < minCosine) {
                minCosine = cosine;
            }
        }
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

    public void setAngle(Rotation2d angle) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setSteerAngle(angle);
        }
    }

    public void setRSetpoint(Rotation2d angle) {
        rPIDSetpoint = angle.getRadians();
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
        // If (joystick is actuated): delete setpoint and use joystick control
        // Else if (not rotating) or (setpoint set):
        // .... if (not robot is rotating) and (setpoint not set):
        // .... .... set setpoint to current angle
        // .... use PID control
        // Else: command 0
        boolean vControl = Math.abs(vr) > 1e-3;
        boolean setpointSet = !Double.isNaN(rPIDSetpoint);
        boolean rotating = Math.abs(gyroDataArray[0]) >= 5;
        if (vControl) {
            rPIDSetpoint = Double.NaN;
            // vr = vr;
        } else if (setpointSet || !rotating) {
            if (!rotating && !setpointSet) {
                // TODO: better setpoint logic (include snapping?)
                rPIDSetpoint = getRobotAngle().getRadians();
            }
            rController.setSetpoint(rPIDSetpoint);
            vr = rController.calculate(getRobotAngle().getRadians());
        } else {
            vr = 0;
            // rPIDSetpoint = Double.NaN;
        }

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vr, getRobotAngle());

        SmartDashboard.putBoolean("Rotating", rotating);
        SmartDashboard.putBoolean("Setpoint", setpointSet);
        SmartDashboard.putBoolean("vControl", vControl);
        SmartDashboard.putNumber("Vx", vx);
        SmartDashboard.putNumber("Vy", vy);
        SmartDashboard.putNumber("Vr", vr);
        SmartDashboard.putNumber("rPIDSetpoint", Math.toDegrees(rPIDSetpoint));

        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, speeds, MAXIMUM_LINEAR_VELOCITY,
                MAXIMUM_LINEAR_VELOCITY, MAXIMUM_ANGULAR_VELOCITY);
        setStates(newStates);
    }

    /**
     * @return the current estimated robot rotation in the range (-π, π)
     */
    public Rotation2d getRobotAngle() {
        return poseEstimator.getEstimatedPosition()
                            .getRotation();
    }

    /**
     * @return the current yaw reading of the {@link Pigeon2} IMU in the range
     *         (-π, π)
     */
    public Rotation2d getGyroAngle() {
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
        poseEstimator.update(Rotation2d.fromDegrees(gyro.getYaw()), positions);
    }

    public double getRSetpoint() {
        return rPIDSetpoint;
    }

    @Override
    public void periodic() {
        updatePositions();
        gyro.getRawGyro(gyroDataArray);

        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        field2d.setRobotPose(currentPose);
        SmartDashboard.putNumber("Pos X", currentPose.getX());
        SmartDashboard.putNumber("Pos Y", currentPose.getY());
        SmartDashboard.putNumber("Pos Rot", currentPose.getRotation()
                                                       .getDegrees());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
    }

    /**
     * @return a reference to the internal array of {@link SwerveModule
     *         SwerveModules}
     */
    public SwerveModule[] getModules() {
        return modules;
    }

    public void displayTrajectory(SwerveTrajectory trajectory) {
        field2d.getObject("trajectory")
               .setTrajectory(trajectory.toTrajectory());
    }
}
