package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.ACCELERATION_ELBOW;
import static frc.robot.Constants.Arm.ELBOW_SEGMENT_CENTER_OF_MASS;
import static frc.robot.Constants.Arm.ELBOW_SEGMENT_EFFICIENCY;
import static frc.robot.Constants.Arm.ELBOW_SEGMENT_LENGTH;
import static frc.robot.Constants.Arm.ELBOW_SEGMENT_MASS;
import static frc.robot.Constants.Arm.INV_ELBOW_GEAR_RATIO;
import static frc.robot.Constants.Arm.MAXIMUM_VELOCITY_ELBOW;
import static frc.robot.Constants.Arm.kD_ELBOW;
import static frc.robot.Constants.Arm.kIZ_ELBOW;
import static frc.robot.Constants.Arm.kI_ELBOW;
import static frc.robot.Constants.Arm.kP_ELBOW;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_ELBOW;
import static frc.robot.Constants.Wiring.ELBOW_CANCODER_ID;

public class ArmElbow extends ArmSegment {
    private static final double[] elbowPos = { -87, -80, -28, 16, -30, 30, 45, 60, 75, 90 };

    public ArmElbow() {
        super("Elbow", ARM_MOTOR_ID_ELBOW, ELBOW_CANCODER_ID, kP_ELBOW, kI_ELBOW, kD_ELBOW,
                kIZ_ELBOW, INV_ELBOW_GEAR_RATIO, elbowPos, ELBOW_SEGMENT_EFFICIENCY,
                MAXIMUM_VELOCITY_ELBOW, ACCELERATION_ELBOW, ELBOW_SEGMENT_MASS,
                ELBOW_SEGMENT_LENGTH, ELBOW_SEGMENT_CENTER_OF_MASS);
    }
}
