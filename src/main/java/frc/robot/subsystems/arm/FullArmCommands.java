package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmElbow;
import frc.robot.subsystems.arm.ArmWrist;

public class FullArmCommands{
    private ArmBase base;
    private ArmElbow elbow;
    private ArmWrist wrist;

    public FullArmCommands(ArmBase base, ArmElbow elbow, ArmWrist wrist){
        this.base = base;
        this.elbow = elbow;
        this.wrist = wrist;
    }
    public ParallelCommandGroup armToPos1 = new ParallelCommandGroup(
        new Command setBaseAngleCommandPos(0, base),
        new Command setElbowAngleCommandPos(0, elbow),
        new Command setWristAngleCommandPos(0, wrist)
    );
}
