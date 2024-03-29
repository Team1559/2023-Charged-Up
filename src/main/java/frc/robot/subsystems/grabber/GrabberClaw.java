package frc.robot.subsystems.grabber;

import static frc.robot.Constants.Grabber.CLAW_PNEUMATIC_WAIT_TIME;
import static frc.robot.Constants.Wiring.CLAW_SOLENOID_ID;
import static frc.robot.Constants.Wiring.PNEUMATICS_HUB_ID;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class GrabberClaw extends SubsystemBase {
    private Solenoid clawSolenoid;

    public GrabberClaw() {
        clawSolenoid = new Solenoid(PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH,
                CLAW_SOLENOID_ID);
    }

    public void openClaw() {
        clawSolenoid.set(true);
    }

    private void closeClaw() {
        clawSolenoid.set(false);
    }

    public Command openClawCommand() {
        return new InstantCommand(this::openClaw, this).andThen(new WaitCommand(CLAW_PNEUMATIC_WAIT_TIME));
    }

    public Command closeClawCommand() {
        return new InstantCommand(this::closeClaw, this).andThen(new WaitCommand(
                CLAW_PNEUMATIC_WAIT_TIME));
    }

    public boolean clawIsOpen() {
        return clawSolenoid.get();
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            closeClaw();
        }
    }
}
