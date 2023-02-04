package frc.robot.subsystems.grabber;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static frc.robot.Constants.Grabber.CLAW_PNEUMATIC_WAIT_TIME;
import static frc.robot.Constants.Grabber.FIRST_DOUBLE_SOLENOID_CHANNEL;
import static frc.robot.Constants.Grabber.SECOND_DOUBLE_SOLENOID_CHANNEL;
import static frc.robot.Constants.Wiring.CLAW_SOLENOID_ID;
import static frc.robot.Constants.Wiring.PDH_ID;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class GrabberClaw extends SubsystemBase {
    // pneumatic solenoid
    private Solenoid clawSolenoid;
    private DoubleSolenoid clawPressure = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FIRST_DOUBLE_SOLENOID_CHANNEL, SECOND_DOUBLE_SOLENOID_CHANNEL);

    public GrabberClaw() {
        clawSolenoid = new Solenoid(PDH_ID, PneumaticsModuleType.REVPH,
                CLAW_SOLENOID_ID);
        clawPressure = new DoubleSolenoid(PneumaticsModuleType.REVPH, FIRST_DOUBLE_SOLENOID_CHANNEL, SECOND_DOUBLE_SOLENOID_CHANNEL);
    }

    public void openClaw() {
        clawSolenoid.set(true);
        clawPressure.set(kForward);
    }

    public void grabCone(){
        closeClaw(true);
    }

    public void grabCube(){
        closeClaw(false);
    }
    private void closeClaw(boolean pressure) {
        clawPressure.set(kReverse);

        clawSolenoid.set(false);
    }

    public Command openClawCommand() { //bound to a button in robotContainer
        return Commands.sequence(
                new InstantCommand(() -> openClaw(), this),
                new WaitCommand(CLAW_PNEUMATIC_WAIT_TIME));
    }

    public Command closeClawCUBECommand() { //bound to a button in robotContainer
        return Commands.sequence(
                new InstantCommand(() -> grabCube(), this),
                new WaitCommand(CLAW_PNEUMATIC_WAIT_TIME));
    }

    public Command closeClawCONECommand() { //bound to a button in robotContainer
        return Commands.sequence(
                new InstantCommand(() -> grabCone(), this),
                new WaitCommand(CLAW_PNEUMATIC_WAIT_TIME));
    }
    
    public String clawState() {
        if (clawPressure.get() == kReverse && clawSolenoid.get() == false) {
            return "Cone";
        } else if (clawSolenoid.get() && clawPressure.get() == kOff) {
            return "Claw Open";
        } else {
            return "Cube";
        }
    }

    public Value pressureState() {
        return clawPressure.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Claw Status", clawState());
    }
}