package frc.robot.subsystems.swerve;

import static frc.robot.Constants.FeatureFlags.VISION_ENABLED;
import static frc.robot.Constants.Swerve.ENCODER_STDDEV;
import static frc.robot.Constants.Swerve.MAXIMUM_ANGULAR_VELOCITY;
import static frc.robot.Constants.Swerve.MAXIMUM_LINEAR_VELOCITY;
import static frc.robot.Constants.Swerve.MAX_ACCEL_PER_CYCLE_R;
import static frc.robot.Constants.Swerve.MAX_ACCEL_PER_CYCLE_X;
import static frc.robot.Constants.Swerve.MAX_ACCEL_PER_CYCLE_Y;
import static frc.robot.Constants.Swerve.MODULE_X;
import static frc.robot.Constants.Swerve.MODULE_Y;
import static frc.robot.Constants.Swerve.ROTATION_KP;
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
import edu.wpi.first.wpilibj.DriverStation;
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
    private boolean                        isFieldRelative;
    private Field2d                        field2d;
    private double                         rPIDSetpoint;
    private double                         lastVX;
    private double                         lastVY;
    private double                         lastVR;

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
        rController.setTolerance(Math.toRadians(1));
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

    public void driveVelocity(ChassisSpeeds speeds) {
        driveVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond);
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
        boolean setpointSet = !Double.isNaN(rPIDSetpoint);
        boolean rotating = Math.abs(gyroDataArray[0]) >= 5;
        if (DriverStation.isTeleop() || !VISION_ENABLED) {
            rPIDSetpoint = Double.NaN;
            // vr = vr;
        } else if (setpointSet || !rotating) {
            if (!rotating && !setpointSet) {
                // TODO: better setpoint logic (include snapping?)
                rPIDSetpoint = getRobotAngle().getRadians();
            }
            rController.setSetpoint(rPIDSetpoint);
            vr = rController.calculate(getRobotAngle().getRadians());
            if (rController.atSetpoint()) {
                vr = 0;
            } else if (Math.abs(vr) > MAXIMUM_ANGULAR_VELOCITY) {
                vr = Math.copySign(MAXIMUM_ANGULAR_VELOCITY, vr);
            }
        } else {
            vr = 0;
            // rPIDSetpoint = Double.NaN;
        }

        ChassisSpeeds speeds;
        if (isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vr, getRobotAngle());
        } else {
            speeds = new ChassisSpeeds(vx, vy, vr);
        }

        // Accel limits
        if (speeds.vxMetersPerSecond > lastVX + MAX_ACCEL_PER_CYCLE_X) {
            speeds.vxMetersPerSecond = lastVX + MAX_ACCEL_PER_CYCLE_X;
        } else if (speeds.vxMetersPerSecond < lastVX - MAX_ACCEL_PER_CYCLE_X) {
            speeds.vxMetersPerSecond = lastVX - MAX_ACCEL_PER_CYCLE_X;
        }
        if (speeds.vyMetersPerSecond > lastVY + MAX_ACCEL_PER_CYCLE_Y) {
            speeds.vyMetersPerSecond = lastVY + MAX_ACCEL_PER_CYCLE_Y;
        } else if (speeds.vyMetersPerSecond < lastVY - MAX_ACCEL_PER_CYCLE_Y) {
            speeds.vyMetersPerSecond = lastVY - MAX_ACCEL_PER_CYCLE_Y;
        }
        if (speeds.omegaRadiansPerSecond > lastVR + MAX_ACCEL_PER_CYCLE_R) {
            speeds.omegaRadiansPerSecond = lastVR + MAX_ACCEL_PER_CYCLE_R;
        } else if (speeds.omegaRadiansPerSecond < lastVR - MAX_ACCEL_PER_CYCLE_R) {
            speeds.omegaRadiansPerSecond = lastVR - MAX_ACCEL_PER_CYCLE_R;
        }
        lastVX = speeds.vxMetersPerSecond;
        lastVY = speeds.vyMetersPerSecond;
        lastVR = speeds.omegaRadiansPerSecond;

        SmartDashboard.putNumber("Vx", vx);
        SmartDashboard.putNumber("Vy", vy);
        SmartDashboard.putNumber("Vr", vr);
        SmartDashboard.putNumber("rPID error", rController.getPositionError());
        SmartDashboard.putNumber("rPIDSetpoint", Math.toDegrees(rPIDSetpoint));
        SmartDashboard.putBoolean("isFieldRelative", isFieldRelative);

        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, speeds, MAXIMUM_LINEAR_VELOCITY,
                MAXIMUM_LINEAR_VELOCITY, MAXIMUM_ANGULAR_VELOCITY);
        setStates(newStates);
    }

    public void initialize() {
        for (int i = 0; i <= 60; i++) {
            try {
                for (SwerveModule module : modules) {
                    module.initialize();
                }
                System.out.println("Swerve init successful");
                return;
            } catch (IllegalStateException e) {
                System.out.printf("Swerve init failed, retrying... (%d)%n", i);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e2) {
                    // ignore
                }
            }
        }
        throw new IllegalStateException("Swerve init unsuccessful after 30 seconds, restarting...");
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
            modules[i].log();
        }
        poseEstimator.update(Rotation2d.fromDegrees(gyro.getYaw()), positions);
    }

    public void holdPosition() {
        // Wheels make an X
        modules[0].holdPosition(Rotation2d.fromDegrees(45));
        modules[1].holdPosition(Rotation2d.fromDegrees(-45));
        modules[2].holdPosition(Rotation2d.fromDegrees(-45));
        modules[3].holdPosition(Rotation2d.fromDegrees(45));
    }

    public void setFieldRelative() {
        isFieldRelative = true;
    }

    public void setRobotRelative() {
        isFieldRelative = false;
    }

    public boolean isFieldRelative() {
        return isFieldRelative;
    }

    public double getRSetpoint() {
        return rPIDSetpoint;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            rPIDSetpoint = Double.NaN;
        }

        updatePositions();
        gyro.getRawGyro(gyroDataArray);

        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        field2d.setRobotPose(currentPose);
        SmartDashboard.putNumber("Pos X", currentPose.getX());
        SmartDashboard.putNumber("Pos Y", currentPose.getY());
        SmartDashboard.putNumber("Pos Rot", currentPose.getRotation()
                                                       .getDegrees());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
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
