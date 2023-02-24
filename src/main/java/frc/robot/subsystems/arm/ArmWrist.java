package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_WRIST;
import static frc.robot.Constants.Wiring.WRIST_CANCODER_ID;

public class ArmWrist extends ArmSegment {
    private static final double[] wristPos = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    public ArmWrist() {
        super("Wrist", ARM_MOTOR_ID_WRIST, WRIST_CANCODER_ID, ARM_WRIST_CC_OFFSET, kP_WRIST,
                kI_WRIST, kD_WRIST, kIZ_WRIST, INV_ARM_WRIST_GEAR_RATIO, wristPos,
                WRIST_SEGMENT_EFFICIENCY, MAXIMUM_VELOCITY_WRIST, ACCELERATION_WRIST,
                WRIST_SEGMENT_MASS, WRIST_SEGMENT_LENGTH, WRIST_SEGMENT_CENTER_OF_MASS);
    }
}
