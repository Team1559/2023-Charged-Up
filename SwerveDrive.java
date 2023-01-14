package frc.robot;

import static frc.robot.Wiring.BL_MODULE_CANCODER;
import static frc.robot.Wiring.BL_MODULE_DRIVE_MOTOR;
import static frc.robot.Wiring.BL_MODULE_TURN_MOTOR;
import static frc.robot.Wiring.BR_MODULE_CANCODER;
import static frc.robot.Wiring.BR_MODULE_DRIVE_MOTOR;
import static frc.robot.Wiring.BR_MODULE_TURN_MOTOR;
import static frc.robot.Wiring.FL_MODULE_CANCODER;
import static frc.robot.Wiring.FL_MODULE_DRIVE_MOTOR;
import static frc.robot.Wiring.FL_MODULE_TURN_MOTOR;
import static frc.robot.Wiring.FR_MODULE_CANCODER;
import static frc.robot.Wiring.FR_MODULE_DRIVE_MOTOR;
import static frc.robot.Wiring.FR_MODULE_TURN_MOTOR;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {
    // TODO: measure offsets
    private static final double              TRACKWIDTH_METERS                 = Units.inchesToMeters(
            24);
    private static final double              WHEELBASE_METERS                  = Units.inchesToMeters(
            24);
    private static final double              INPUT_THRESHOLD                   = 0.01D;
    private static final GearRatio           GEAR_RATIO                        = Mk4SwerveModuleHelper.GearRatio.L2;
    private static final ModuleConfiguration GEAR_CONFIG                       = GEAR_RATIO.getConfiguration();
    private static final double              MAX_VOLTAGE                       = 12;
    private static final double              MAX_DRIVE_VELOCITY_MPS            = 6380D
            / 60 * Math.PI * GEAR_CONFIG.getDriveReduction()
            * GEAR_CONFIG.getWheelDiameter();
    private static final double              MAX_TURN_SPEED_RADIANS_PER_SECOND = MAX_DRIVE_VELOCITY_MPS
            / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    private static final Translation2d[] MODULE_POS_OFFSETS = {
            new Translation2d(TRACKWIDTH_METERS / 2, WHEELBASE_METERS / 2),
            new Translation2d(TRACKWIDTH_METERS / 2, -WHEELBASE_METERS / 2),
            new Translation2d(-TRACKWIDTH_METERS / 2, WHEELBASE_METERS / 2),
            new Translation2d(-TRACKWIDTH_METERS / 2, -WHEELBASE_METERS / 2) };

    // TODO: find experimental cancoder offsets
    // No need to do -Math.toRadians(), it is handled in createModule()
    private static final double[] CANCODER_OFFSETS = { 70.2, 42.1, 94.3,
            180.9 };
    private static final int[]    DRIVE_MOTOR_IDS  = { FL_MODULE_DRIVE_MOTOR,
            FR_MODULE_DRIVE_MOTOR, BL_MODULE_DRIVE_MOTOR,
            BR_MODULE_DRIVE_MOTOR };
    private static final int[]    TURN_MOTOR_IDS   = { FL_MODULE_TURN_MOTOR,
            FR_MODULE_TURN_MOTOR, BL_MODULE_TURN_MOTOR, BR_MODULE_TURN_MOTOR };
    private static final int[]    CANCODER_IDS     = { FL_MODULE_CANCODER,
            FR_MODULE_CANCODER, BL_MODULE_CANCODER, BR_MODULE_CANCODER };

    // FL, FR, BL, BR
    private final SwerveModule[]        modules;
    private final CANCoder[]            cancoders;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry   odometry;
    // private final AHRS navX;
    private final Pigeon2 pigeon;

    public SwerveDrive() {
    // public SwerveDrive(AHRS navx) {
        // this.navX = navx;
        this.pigeon = new Pigeon2(Wiring.PIGEON_IMU);
        this.modules = new SwerveModule[DRIVE_MOTOR_IDS.length];
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i] = createModule(DRIVE_MOTOR_IDS[i],
                    TURN_MOTOR_IDS[i], CANCODER_IDS[i], CANCODER_OFFSETS[i]);
        }
        this.kinematics = new SwerveDriveKinematics(MODULE_POS_OFFSETS);
        this.odometry = new SwerveDriveOdometry(this.kinematics,
                getCurrentRobotAngle());
        this.cancoders = new CANCoder[] { new CANCoder(FL_MODULE_CANCODER),
                new CANCoder(FR_MODULE_CANCODER),
                new CANCoder(BL_MODULE_CANCODER),
                new CANCoder(BR_MODULE_CANCODER), };
        zeroGyroscope();
    }

    public void drive(double inputX, double inputY, double inputR) {
        double velocityX = inputX * MAX_DRIVE_VELOCITY_MPS;
        double velocityY = inputY * MAX_DRIVE_VELOCITY_MPS;
        double rotation = -inputR * MAX_TURN_SPEED_RADIANS_PER_SECOND;

        // ChassisSpeeds considers +x to be forwards
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocityY,
                -velocityX, rotation, getCurrentRobotAngle());

        SwerveModuleState[] newStates = this.kinematics.toSwerveModuleStates(
                speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates,
                MAX_DRIVE_VELOCITY_MPS);

        double minCosine = 1D;
        for (int i = 0; i < this.modules.length; i++) {
            double cosine = newStates[i].angle.minus(
                    new Rotation2d(this.modules[i].getSteerAngle()))
                                              .getCos();
            SmartDashboard.putNumber("Cosine " + i, cosine);
            if (cosine < minCosine) {
                minCosine = cosine;
            }
        }

        for (int i = 0; i < this.modules.length; i++) {
            // newStates[i].speedMetersPerSecond *= minCosine;
            newStates[i] = SwerveModuleState.optimize(newStates[i],
                    new Rotation2d(this.modules[i].getSteerAngle()));
            this.modules[i].set(newStates[i].speedMetersPerSecond
                    / MAX_DRIVE_VELOCITY_MPS * MAX_VOLTAGE,
                    newStates[i].angle.getRadians());
        }

        SwerveModuleState[] currentStates = Arrays.stream(this.modules)
                                                  .map(m -> new SwerveModuleState(
                                                          m.getDriveVelocity(),
                                                          new Rotation2d(
                                                                  m.getSteerAngle())))
                                                  .toArray(
                                                          SwerveModuleState[]::new);
        this.odometry.update(getCurrentRobotAngle(), currentStates);

        Pose2d estimatedPosition = this.odometry.getPoseMeters();
        SmartDashboard.putNumber("Current X position",
                estimatedPosition.getX());
        SmartDashboard.putNumber("Current Y position",
                estimatedPosition.getY());
        SmartDashboard.putNumber("Current rotation",
                estimatedPosition.getRotation()
                                 .getDegrees());

        for (int i = 0; i < this.cancoders.length; i++) {
            SmartDashboard.putNumber(
                    "Cancoder " + this.cancoders[i].getDeviceID() + " Position",
                    this.cancoders[i].getPosition());
        }
    }

    public double[] getCancoderOffsets() {
        return Arrays.stream(this.cancoders)
                     .mapToDouble(CANCoder::configGetMagnetOffset)
                     .toArray();
    }

    public double[] getCancoderPositions() {
        double[] positions = new double[this.cancoders.length];
        for (int i = 0; i < this.cancoders.length; i++) {
            positions[i] = this.cancoders[i].getAbsolutePosition();
        }
        return positions;
    }

    public void setAngles(double degrees) {
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].set(1.5, Math.toRadians(degrees));
        }
    }

    public void zeroGyroscope() {
        // this.navX.zeroYaw();
        this.pigeon.zeroGyroBiasNow();
    }

    private Rotation2d getCurrentRobotAngle() {
        // return Rotation2d.fromDegrees(0);
        // if (this.navX.isMagnetometerCalibrated()) {
        // return Rotation2d.fromDegrees(navX.getFusedHeading());
        // } else {
        // // We have to invert the angle of the NavX so that rotating the
        // // robot counter-clockwise makes the angle increase.
        // return Rotation2d.fromDegrees(360D - navX.getYaw());
        // }
        return Rotation2d.fromDegrees(this.pigeon.getYaw());
    }

    private static void configureMotor(int driveMotorPort) {
        TalonFX tempMotor = new TalonFX(driveMotorPort);
        tempMotor.configOpenloopRamp(0.5);
    }

    private static SwerveModule createModule(int driveMotorPort,
            int steerMotorPort, int canCoderPort, double steerOffsetDegrees) {
        configureMotor(driveMotorPort);
        return Mk4SwerveModuleHelper.createFalcon500(GEAR_RATIO, driveMotorPort,
                steerMotorPort, canCoderPort,
                -Math.toRadians(steerOffsetDegrees));
    }
}
