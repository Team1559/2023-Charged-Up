// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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
    public static final double FALCON_TICKS_PER_REV = 2048;
    public static final double FALCON_MAX_RPM       = 6380;

    public static class Wiring {
        // Swerve drive
        // Modules are ordered as [ FL, FR, BL, BR ] in arrays
        public static final int[] MODULE_DRIVE_MOTOR_IDS = { 10, 1, 7, 4 };
        public static final int[] MODULE_STEER_MOTOR_IDS = { 12, 3, 9, 6 };
        public static final int[] MODULE_CANCODER_IDS    = { 11, 2, 8, 5 };
        public static final int   PIGEON_IMU             = 0;
        public static final int ARM_MOTOR_ID_BASE = 9997;
        public static final int ARM_MOTOR_ID_ELBOW = 9998;
    }

    public static class Swerve {
        // SDS MK4 L2 Falcons
        public static final double DRIVE_GEAR_RATIO     = (14D / 50D)
                * (27D / 17D) * (15D / 45D);
        public static final double STEER_GEAR_RATIO     = (15D / 32D)
                * (10D / 60D);
        public static final double DRIVE_GEAR_RATIO_INV = 1D / DRIVE_GEAR_RATIO;
        public static final double STEER_GEAR_RATIO_INV = 1D / STEER_GEAR_RATIO;

        public static final double WHEELBASE_WIDTH     = Units.inchesToMeters(
                24);
        public static final double WHEELBASE_LENGTH    = Units.inchesToMeters(
                24);
        public static final double MODULE_X            = WHEELBASE_LENGTH / 2;
        public static final double MODULE_Y            = WHEELBASE_WIDTH / 2;
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(
                3.95) * Math.PI;

        public static final double MAXIMUM_LINEAR_VELOCITY  = Units.feetToMeters(
                16.3);
        public static final double MAXIMUM_ANGULAR_VELOCITY = MAXIMUM_LINEAR_VELOCITY
                / Math.hypot(MODULE_X, MODULE_Y);

        public static final double MINIMUM_LINEAR_VELOCITY  = 0.05;
        public static final double MINIMUM_ANGULAR_VELOCITY = 0.01;

        public static final double METERS_TO_TICKS  = DRIVE_GEAR_RATIO_INV
                * FALCON_TICKS_PER_REV / WHEEL_CIRCUMFERENCE;
        public static final double TICKS_TO_METERS  = 1 / METERS_TO_TICKS;
        public static final double DEGREES_TO_TICKS = FALCON_TICKS_PER_REV
                * STEER_GEAR_RATIO_INV / 360;
        public static final double TICKS_TO_DEGREES = 1 / DEGREES_TO_TICKS;

        public static final double[] CANCODER_OFFSETS = { 70.2, 42.1, 94.3,
                180.9 };

        public static final double MODULE_DRIVE_KP = 0.005;
        public static final double MODULE_DRIVE_KF = 0.05;
    }

    public static class Vision {
        public static final String CAMERA_NAME = "OV5647";
        public static final PoseStrategy POSE_STRATEGY = PoseStrategy.AVERAGE_BEST_TARGETS;

        public static final double CAMERA_X = 0;
        public static final double CAMERA_Y = Units.inchesToMeters(16);
        public static final double CAMERA_Z = Units.inchesToMeters(5.5);
        public static final double CAMERA_ANGLE = Math.toRadians(90);
        public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
                new Translation3d(CAMERA_X, CAMERA_Y, CAMERA_Z),
                new Rotation3d(0, 0, CAMERA_ANGLE));
    }
    public static class Arm{
        public static final double GEAR_RATIO_BASE = (1 / 64.0) * (50/72.0); 
        public static final double INV_GEAR_RATIO_BASE = 1 / GEAR_RATIO_BASE;
        
        public static final double kP_BASE = 0; //All PDI constants will be changed in the future.
        public static final double kD_BASE = 0;
        public static final double kI_BASE = 0; 
        public static final double kF_BASE = 0;

        public static final double kP_ELBOW = 0;
        public static final double kD_ELBOW = 0;
        public static final double kI_ELBOW = 0; 
        public static final double kF_ELBOW = 0;
    }
}
