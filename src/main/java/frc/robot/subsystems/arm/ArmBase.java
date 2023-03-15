package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.ACCELERATION_BASE;
import static frc.robot.Constants.Arm.BASE_CLOSED_LOOP_ERROR;
import static frc.robot.Constants.Arm.BASE_SEGMENT_CENTER_OF_MASS;
import static frc.robot.Constants.Arm.BASE_SEGMENT_EFFICIENCY;
import static frc.robot.Constants.Arm.BASE_SEGMENT_LENGTH;
import static frc.robot.Constants.Arm.BASE_SEGMENT_MASS;
import static frc.robot.Constants.Arm.INV_GEAR_RATIO_BASE;
import static frc.robot.Constants.Arm.LOWER_BASE_LIMIT;
import static frc.robot.Constants.Arm.MAXIMUM_VELOCITY_BASE;
import static frc.robot.Constants.Arm.UPPER_BASE_LIMIT;
import static frc.robot.Constants.Arm.kD_BASE;
import static frc.robot.Constants.Arm.kIZ_BASE;
import static frc.robot.Constants.Arm.kI_BASE;
import static frc.robot.Constants.Arm.kP_BASE;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_BASE;
import static frc.robot.Constants.Wiring.BASE_CANCODER_ID;

import frc.robot.subsystems.arm.Arm.Position;

public class ArmBase extends ArmSegment {

    public ArmBase() {
        super("Base", ARM_MOTOR_ID_BASE, BASE_CANCODER_ID, kP_BASE, kI_BASE, kD_BASE, kIZ_BASE,
                INV_GEAR_RATIO_BASE, BASE_SEGMENT_EFFICIENCY, MAXIMUM_VELOCITY_BASE,
                ACCELERATION_BASE, BASE_SEGMENT_MASS, BASE_SEGMENT_LENGTH,
                BASE_SEGMENT_CENTER_OF_MASS, false, LOWER_BASE_LIMIT, UPPER_BASE_LIMIT,
                BASE_CLOSED_LOOP_ERROR);
    }

    @Override
    protected double getTargetAngle(Position position) {
        return position.base;
    }
}
