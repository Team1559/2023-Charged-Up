package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_WRIST;
import static frc.robot.Constants.Wiring.WRIST_CANCODER_ID;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmWrist extends ArmSegment {
    private static final double[] wristPos = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    public ArmWrist() {
        super("Wrist", ARM_MOTOR_ID_WRIST, WRIST_CANCODER_ID, kP_WRIST,
                kI_WRIST, kD_WRIST, kIZ_WRIST, INV_ARM_WRIST_GEAR_RATIO,
                new ArmFeedforward(kS_WRIST, kG_WRIST, kV_WRIST, kA_WRIST),
                wristPos);
    }
}
