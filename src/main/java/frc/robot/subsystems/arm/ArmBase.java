package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_BASE;
import static frc.robot.Constants.Wiring.BASE_CANCODER_ID;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmBase extends ArmSegment {
    private static final double[] basePos = { 0, 10, 20, 30, 40, 50, 60, 70, 80,
            90 };

    public ArmBase() {
        super("Base", ARM_MOTOR_ID_BASE, BASE_CANCODER_ID, kP_BASE, kI_BASE,
                kD_BASE, kIZ_BASE, INV_GEAR_RATIO_BASE,
                new ArmFeedforward(kS_BASE, kG_BASE, kV_BASE, kA_BASE),
                basePos, null);
    }
}
