package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmElbowPositionCommands extends CommandBase{
    private ArmElbow elbow;
    private double angle;

    private final double angleVel = 50D;
    private final double angleVelPerCycle = angleVel / 50.0;

    public ArmElbowPositionCommands(ArmElbow elbow, double angle){
        this.elbow = elbow;
        this.angle = angle;
    }

    @Override
    public void initialize(){
        elbow.setAngle(angle);
    }
    @Override 
    public boolean isFinished(){
        double actualangle = Math.abs(angle - elbow.getAngle()); //Gets error w/o using PID, at least for the mooment
        if (actualangle < 0.05){
            return true;
        } else {
            return false;
        }
    }
}