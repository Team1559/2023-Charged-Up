package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmBasePositionCommands extends CommandBase{
    private ArmBase base;
    private double angle;

    public ArmBasePositionCommands(ArmBase base, double angle){
        this.base = base;
        this.angle = angle;
    }

    @Override
    public void initialize(){
        base.setAngle(angle);
    }
    @Override 
    public boolean isFinished(){
        double actualangle = Math.abs(angle - base.getAngle());
        if (actualangle < 0.05){
            return true;
        } else {
            return false;
        }
    }
}