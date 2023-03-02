package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.ACCELERATION_BASE;
import static frc.robot.Constants.Arm.BASE_SEGMENT_CENTER_OF_MASS;
import static frc.robot.Constants.Arm.BASE_SEGMENT_EFFICIENCY;
import static frc.robot.Constants.Arm.BASE_SEGMENT_LENGTH;
import static frc.robot.Constants.Arm.BASE_SEGMENT_MASS;
import static frc.robot.Constants.Arm.INV_GEAR_RATIO_BASE;
import static frc.robot.Constants.Arm.MAXIMUM_VELOCITY_BASE;
import static frc.robot.Constants.Arm.kD_BASE;
import static frc.robot.Constants.Arm.kIZ_BASE;
import static frc.robot.Constants.Arm.kI_BASE;
import static frc.robot.Constants.Arm.kP_BASE;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_BASE;
import static frc.robot.Constants.Wiring.BASE_CANCODER_ID;

public class ArmBase extends ArmSegment {
    private static final double[] basePos = { 68, 48, 52, 47, 30, 50, 60, 70, 80, 90 };

    public ArmBase() {
        super("Base", ARM_MOTOR_ID_BASE, BASE_CANCODER_ID, kP_BASE, kI_BASE, kD_BASE, kIZ_BASE,
                INV_GEAR_RATIO_BASE, basePos, BASE_SEGMENT_EFFICIENCY, MAXIMUM_VELOCITY_BASE,
                ACCELERATION_BASE, BASE_SEGMENT_MASS, BASE_SEGMENT_LENGTH,
                BASE_SEGMENT_CENTER_OF_MASS);
    }
}
