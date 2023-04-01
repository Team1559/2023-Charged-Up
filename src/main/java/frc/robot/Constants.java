// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // LEAVE THESE AT TOP OF FILE
    // NEVER DISABLE THESE IN MASTER BRANCH
    public static class FeatureFlags {
        public static final boolean CHASSIS_ENABLED = RobotBase.isReal();
        public static final boolean ARM_ENABLED     = RobotBase.isReal();
        public static final boolean GRABBER_ENABLED = RobotBase.isReal();
        public static final boolean VISION_ENABLED  = RobotBase.isReal();
    }

    public static final double FALCON_TICKS_PER_REV   = 2048;
    public static final double CANCODER_TICKS_PER_REV = 4096;
    public static final double FALCON_MAX_RPM         = 6380;
    public static final double FALCON_STALL_TORQUE    = 4.69;
    public static final double GRAVITY_ACCELERATION   = 9.8;
    public static final double NOMINAL_VOLTAGE        = 12D;
    public static final double CYCLES_PER_SECOND      = 50D;

    public static class Wiring {
        // Swerve drive
        // Modules are ordered as [ FL, FR, BL, BR ] in arrays
        public static final int[]  MODULE_DRIVE_MOTOR_IDS = { 10, 1, 7, 4 };
        public static final int[]  MODULE_STEER_MOTOR_IDS = { 12, 3, 9, 6 };
        public static final int[]  MODULE_CANCODER_IDS    = { 11, 2, 8, 5 };
        public static final int    PIGEON_IMU             = 0;
        public static final String CANIVORE_BUS_ID        = "1559Canivore";

        // Arm wiring ports + ids
        public static final int ARM_MOTOR_ID_BASE     = 18;
        public static final int ARM_MOTOR_ID_ELBOW    = 17;
        public static final int ARM_MOTOR_ID_WRIST    = 19;
        public static final int BASE_CANCODER_ID      = 2;
        public static final int ELBOW_CANCODER_ID     = 3;
        public static final int ARM_WRIST_CANCODER_ID = 4;
        public static final int PDH_ID                = 34;
        public static final int CLAW_SOLENOID_ID      = 7;
        public static final int PNEUMATICS_HUB_ID     = 31;

        public static final int WRIST_SERVO_PORT = 2;

        // Lighting PWM ports
        public static final int PWM_RED_PORT   = 1;
        public static final int PWM_GREEN_PORT = 3;
        public static final int PWM_BLUE_PORT  = 2;

    }

    public static class Swerve {
        // SDS MK4 L2 Falcons
        public static final double DRIVE_GEAR_RATIO     = (14D / 50D) * (27D / 17D) * (15D / 45D);
        public static final double STEER_GEAR_RATIO     = (15D / 32D) * (10D / 60D);
        public static final double DRIVE_GEAR_RATIO_INV = 1D / DRIVE_GEAR_RATIO;
        public static final double STEER_GEAR_RATIO_INV = 1D / STEER_GEAR_RATIO;
        public static final double STEER_DRIVE_BACKLASH = STEER_GEAR_RATIO * (50D / 14D);

        public static final double WHEELBASE_WIDTH     = Units.inchesToMeters(24);
        public static final double WHEELBASE_LENGTH    = Units.inchesToMeters(24);
        public static final double MODULE_X            = WHEELBASE_LENGTH / 2;
        public static final double MODULE_Y            = WHEELBASE_WIDTH / 2;
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(3.95) * Math.PI;

        public static final double MAXIMUM_LINEAR_VELOCITY  = Units.feetToMeters(16.3);
        public static final double MAXIMUM_ANGULAR_VELOCITY = MAXIMUM_LINEAR_VELOCITY
                / Math.hypot(MODULE_X, MODULE_Y);

        public static final double SWERVE_SECONDS_TO_FULL_SPEED_SLOW = 1.0;
        public static final double MAX_ACCEL_PER_CYCLE_SLOW          = MAXIMUM_LINEAR_VELOCITY
                / SWERVE_SECONDS_TO_FULL_SPEED_SLOW / CYCLES_PER_SECOND;
        public static final double SWERVE_SECONDS_TO_FULL_SPEED_Y    = 0.4;
        public static final double MAX_ACCEL_PER_CYCLE_Y             = MAXIMUM_LINEAR_VELOCITY
                / SWERVE_SECONDS_TO_FULL_SPEED_Y / CYCLES_PER_SECOND;
        public static final double SWERVE_SECONDS_TO_FULL_SPEED_R    = 0.4;
        public static final double MAX_ACCEL_PER_CYCLE_R             = MAXIMUM_ANGULAR_VELOCITY
                / SWERVE_SECONDS_TO_FULL_SPEED_R / CYCLES_PER_SECOND;

        public static final double SLOW_MODE_RATIO = 0.5;

        public static final double MINIMUM_LINEAR_VELOCITY  = 0.05;
        public static final double MINIMUM_ANGULAR_VELOCITY = 0.01;

        public static final double METERS_TO_TICKS  = DRIVE_GEAR_RATIO_INV * FALCON_TICKS_PER_REV
                / WHEEL_CIRCUMFERENCE;
        public static final double TICKS_TO_METERS  = 1 / METERS_TO_TICKS;
        public static final double DEGREES_TO_TICKS = FALCON_TICKS_PER_REV * STEER_GEAR_RATIO_INV
                / 360;
        public static final double TICKS_TO_DEGREES = 1 / DEGREES_TO_TICKS;

        public static final double[] CANCODER_OFFSETS = { 73.3, 44.5, 93.2, 181.4 };

        public static final double MODULE_DRIVE_KP = 0.05;
        public static final double MODULE_STEER_KP = 0.22;
        public static final double MODULE_STEER_KD = 0.1;

        public static final double ROTATION_KP             = 10;
        public static final double ENCODER_STDDEV          = 0.01;
        public static final double ROTATION_SNAP_THRESHOLD = 5;
    }

    public static class Vision {
        public static final String       CAMERA_NAME_FRONT   = "Limelight_2P_Front";
        public static final String       CAMERA_NAME_BACK    = "Limelight_2P_Back";
        public static final PoseStrategy POSE_STRATEGY       = PoseStrategy.LOWEST_AMBIGUITY;
        public static final double       AMBIGUITY_THRESHOLD = 0.2;

        // offsets are in meters
        public static final Transform3d ROBOT_TO_CAMERA_FRONT = new Transform3d(
                new Translation3d(0.31, -0.265, 0.34), new Rotation3d(0, 0, Math.toRadians(0)));
        // x=-15.5 y=-7.5 z=22.3
        public static final Transform3d ROBOT_TO_CAMERA_BACK = new Transform3d(
                new Translation3d(-0.394, -0.19, 0.566), new Rotation3d(0, 0, Math.toRadians(180)));
    }

    public static class Arm {
        public static final double GEAR_RATIO_BASE                 = (1 / 64.0) * (50.0 / 72.0);
        public static final double INV_GEAR_RATIO_BASE             = 1 / GEAR_RATIO_BASE;
        public static final double ELBOW_GEAR_RATIO                = 1 / 64.0;
        public static final double INV_ELBOW_GEAR_RATIO            = 1 / ELBOW_GEAR_RATIO;
        public static final double ARM_WRIST_GEAR_RATIO            = 1 / 64.0;
        public static final double INV_ARM_WRIST_GEAR_RATIO        = 1 / ARM_WRIST_GEAR_RATIO;
        public static final double TELEOP_ANGLE_VELOCITY           = 90D;
        public static final double TELEOP_ANGLE_VELOCITY_PER_CYCLE = TELEOP_ANGLE_VELOCITY
                / CYCLES_PER_SECOND;

        public static final double ANGULAR_VELOCITY_UNIT_DPS = 20D;
        public static final double MAXIMUM_VELOCITY_WRIST    = 120D / CYCLES_PER_SECOND;
        public static final double MAXIMUM_VELOCITY_ELBOW    = 240D / CYCLES_PER_SECOND;
        public static final double MAXIMUM_VELOCITY_BASE     = 120D / CYCLES_PER_SECOND;
        public static final double MINIMUM_TARGET_DISTANCE   = 0.5;

        public static final double ACCELERATION_WRIST = 2.4 / CYCLES_PER_SECOND;
        public static final double ACCELERATION_ELBOW = 2.4 / CYCLES_PER_SECOND;
        public static final double ACCELERATION_BASE  = 1.8 / CYCLES_PER_SECOND;

        public static final double DECELERATION_WRIST = 2.4 / CYCLES_PER_SECOND;
        public static final double DECELERATION_ELBOW = 1.2 / CYCLES_PER_SECOND;
        public static final double DECELERATION_BASE  = 0.8 / CYCLES_PER_SECOND;

        public static final double ZERO_ANGLE          = 0D;
        public static final double MAXIMUM_ANGLE_ERROR = 0.5;

        public static final double kP_BASE  = 1.5;
        public static final double kD_BASE  = 1.0;
        public static final double kI_BASE  = 0.0025; // 0.0015;
        public static final double kIZ_BASE = 30;

        public static final double kP_ELBOW  = 0.7;
        public static final double kD_ELBOW  = 0.2;
        public static final double kI_ELBOW  = 0.0015;
        public static final double kIZ_ELBOW = 55;

        public static final double kP_WRIST  = 1;
        public static final double kD_WRIST  = 0;
        public static final double kI_WRIST  = 0.1;
        public static final double kIZ_WRIST = 3;

        public static final double BASE_SEGMENT_EFFICIENCY  = 0.3; // 0.3;
        public static final double ELBOW_SEGMENT_EFFICIENCY = 1.0; // 0.8;
        public static final double WRIST_SEGMENT_EFFICIENCY = 1.0; // 0.7;

        public static final double BASE_SEGMENT_MASS  = Units.lbsToKilograms(9.5);
        public static final double ELBOW_SEGMENT_MASS = Units.lbsToKilograms(6.5);
        public static final double WRIST_SEGMENT_MASS = Units.lbsToKilograms(6.7);

        public static final double BASE_SEGMENT_LENGTH  = Units.inchesToMeters(32);
        public static final double ELBOW_SEGMENT_LENGTH = Units.inchesToMeters(28);
        public static final double WRIST_SEGMENT_LENGTH = Units.inchesToMeters(21.75);

        public static final Translation2d BASE_SEGMENT_CENTER_OF_MASS  = new Translation2d(
                Units.inchesToMeters(19), 0);
        public static final Translation2d ELBOW_SEGMENT_CENTER_OF_MASS = new Translation2d(
                Units.inchesToMeters(16), 0);
        public static final Translation2d WRIST_SEGMENT_CENTER_OF_MASS = new Translation2d(
                Units.inchesToMeters(11.75), 0);

        public static final double LOWER_ARM_WRIST_LIMIT = -95.0;
        public static final double UPPER_ARM_WRIST_LIMIT = 60.0;
        public static final double LOWER_ELBOW_LIMIT     = -154.0;
        public static final double UPPER_ELBOW_LIMIT     = 0.0;
        public static final double LOWER_BASE_LIMIT      = 50.0;
        public static final double UPPER_BASE_LIMIT      = 100.5;

        public static final double BASE_CLOSED_LOOP_ERROR  = 1;
        public static final double ELBOW_CLOSED_LOOP_ERROR = 1;
        public static final double WRIST_CLOSED_LOOP_ERROR = 1;
    }

    public static class Grabber {
        public static final double ZERO_ANGLE                        = 103D;
        public static final double SERVO_RANGE                       = 180D;
        public static final double TELEOP_ANGULAR_VELOCITY           = 120D;
        public static final double TELEOP_ANGULAR_VELOCITY_PER_CYCLE = TELEOP_ANGULAR_VELOCITY / 50;
        public static final double MAX_ANGULAR_VELOCITY              = 360D;
        public static final double CLAW_PNEUMATIC_WAIT_TIME          = 0.1;
        public static final double MINIMUM_WRIST_ANGLE               = -90D;
        public static final double MAXIMUN_WRIST_ANGLE               = 90D;
        public static final double RESET_WRIST_ANGLE                 = 0;

    }

    public static class Auto {
        public static final double MAXIMUM_LINEAR_VELOCITY  = 2;
        public static final double MAXIMUM_ANGULAR_VELOCITY = 1 * Math.PI;

        public static final double ACCELERATION_TIME            = 0.5;
        public static final double MAXIMUM_LINEAR_ACCELERATION  = MAXIMUM_LINEAR_VELOCITY
                / ACCELERATION_TIME;
        public static final double MAXIMUM_ANGULAR_ACCELERATION = MAXIMUM_ANGULAR_VELOCITY
                / ACCELERATION_TIME;

        public static final double LINEAR_TOLERANCE   = 0.05;
        public static final double ANGULAR_TOLERANCE  = 2;
        public static final double LOOKAHEAD_DISTANCE = 0.1;

        public static final double DISTANCE_BETWEEN_POINTS = 0.025;
        public static final double SMOOTH_TOLERANCE        = 0.001;
        public static final double SMOOTH_WEIGHT           = 0.75;
        public static final double VELOCITY_PROPORTION     = 5;
        public static final double VELOCITY_POWER          = 0.2;
    }
}
