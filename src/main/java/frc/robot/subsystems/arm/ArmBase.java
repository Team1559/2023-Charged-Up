package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_BASE;
import static frc.robot.Constants.Wiring.BASE_CANCODER_ID;

public class ArmBase extends ArmSegment {
    private static final double[] basePos = { 91, 90, 63, 83, 30, 50, 60, 70, 80, 90 };

    public ArmBase() {
        super("Base", ARM_MOTOR_ID_BASE, BASE_CANCODER_ID, kP_BASE, kI_BASE, kD_BASE, kIZ_BASE,
                INV_GEAR_RATIO_BASE, basePos, BASE_SEGMENT_EFFICIENCY, MAXIMUM_VELOCITY_BASE,
                ACCELERATION_BASE, BASE_SEGMENT_MASS, BASE_SEGMENT_LENGTH,
                BASE_SEGMENT_CENTER_OF_MASS, false);
    }
}
