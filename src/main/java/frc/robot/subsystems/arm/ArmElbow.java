package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.ELBOW_SEGMENT_CENTER_OF_MASS;
import static frc.robot.Constants.Arm.ELBOW_SEGMENT_EFFICIENCY;
import static frc.robot.Constants.Arm.ELBOW_SEGMENT_LENGTH;
import static frc.robot.Constants.Arm.ELBOW_SEGMENT_MASS;
import static frc.robot.Constants.Arm.INV_GEAR_RATIO_BASE;
import static frc.robot.Constants.Arm.kD_ELBOW;
import static frc.robot.Constants.Arm.kIZ_ELBOW;
import static frc.robot.Constants.Arm.kI_ELBOW;
import static frc.robot.Constants.Arm.kP_ELBOW;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_ELBOW;
import static frc.robot.Constants.Wiring.ELBOW_CANCODER_ID;

public class ArmElbow extends ArmSegment {
    private static final double[] elbowPos = { 0, 0, 0, 0, 0, -60, -45, -30,
            -15, 0 };

    public ArmElbow() {
        super("Elbow", ARM_MOTOR_ID_ELBOW, ELBOW_CANCODER_ID, kP_ELBOW,
                kI_ELBOW, kD_ELBOW, kIZ_ELBOW, INV_GEAR_RATIO_BASE, elbowPos,
                ELBOW_SEGMENT_EFFICIENCY, ELBOW_SEGMENT_MASS,
                ELBOW_SEGMENT_LENGTH, ELBOW_SEGMENT_CENTER_OF_MASS);
    }
}
