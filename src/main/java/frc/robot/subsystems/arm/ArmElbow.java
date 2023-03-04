package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_ELBOW;
import static frc.robot.Constants.Wiring.ELBOW_CANCODER_ID;

public class ArmElbow extends ArmSegment {
    private static final double[] elbowPos = { -140, -113, -50, -88, -52, 30, 45, 60, 75, 90 };

    public ArmElbow() {
        super("Elbow", ARM_MOTOR_ID_ELBOW, ELBOW_CANCODER_ID, kP_ELBOW, kI_ELBOW, kD_ELBOW,
                kIZ_ELBOW, INV_ELBOW_GEAR_RATIO, elbowPos, ELBOW_SEGMENT_EFFICIENCY,
                MAXIMUM_VELOCITY_ELBOW, ACCELERATION_ELBOW, ELBOW_SEGMENT_MASS,
                ELBOW_SEGMENT_LENGTH, ELBOW_SEGMENT_CENTER_OF_MASS, true, LOWER_ELBOW_LIMIT,
                UPPER_ELBOW_LIMIT, ELBOW_CLOSED_LOOP_ERROR);
    }
}
// -145 -> 0