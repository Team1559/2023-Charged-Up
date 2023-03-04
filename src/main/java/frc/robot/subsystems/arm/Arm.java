package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Arm {
    private ArmBase  base;
    private ArmElbow elbow;
    private ArmWrist wrist;

    public Arm(ArmBase base, ArmElbow elbow, ArmWrist wrist) {
        this.base = base;
        this.elbow = elbow;
        this.wrist = wrist;
    }

    private Command moveArmToPosition(int index) {
        return new ParallelCommandGroup(base.setAngleCommandPos(index),
                elbow.setAngleCommandPos(index), wrist.setAngleCommandPos(index));
    }

    public SequentialCommandGroup moveToLocations(int... positions) {
        Command[] groupOfCommands = new Command[positions.length];
        for (int i = 0; i < groupOfCommands.length; i++) {
            groupOfCommands[i] = moveArmToPosition(positions[i]);
        }
        return new SequentialCommandGroup(groupOfCommands);
    }

}