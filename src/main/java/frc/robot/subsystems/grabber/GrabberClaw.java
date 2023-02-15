package frc.robot.subsystems.grabber;

import static frc.robot.Constants.Grabber.CLAW_PNEUMATIC_WAIT_TIME;
import static frc.robot.Constants.Wiring.CLAW_SOLENOID_ID;
import static frc.robot.Constants.Wiring.PDH_ID;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class GrabberClaw extends SubsystemBase {
    private Solenoid clawSolenoid;

    public GrabberClaw() {
        clawSolenoid = new Solenoid(PDH_ID, PneumaticsModuleType.REVPH,
                CLAW_SOLENOID_ID);
    }

    public void openClaw() {
        clawSolenoid.set(true);
    }

    private void closeClaw() {
        clawSolenoid.set(false);
    }

    public Command openClawCommand() { // bound to a button in robotContainer
        return Commands.sequence(new InstantCommand(this::openClaw, this),
                new WaitCommand(CLAW_PNEUMATIC_WAIT_TIME));
    }

    public Command closeClawCommand() {
        return Commands.sequence(new InstantCommand(this::closeClaw, this),
                new WaitCommand(CLAW_PNEUMATIC_WAIT_TIME));
    }

    public boolean clawIsOpen() {
        return clawSolenoid.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Claw Closed", !clawIsOpen());
    }
}
