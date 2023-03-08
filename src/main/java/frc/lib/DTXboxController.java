package frc.lib;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DTXboxController {
    public enum RumbleSide {
        LEFT(true, false),
        RIGHT(false, true),
        BOTH(true, true);

        private final boolean isLeft;
        private final boolean isRight;

        RumbleSide(boolean isLeft, boolean isRight) {
            this.isLeft = isLeft;
            this.isRight = isRight;
        }
    }

    private class RumbleTask extends TimerTask {
        @Override
        public void run() {
            long currentTime = System.currentTimeMillis();
            if (currentTime >= leftTimeout) {
                leftPower = 0;
            }
            if (currentTime >= rightTimeout) {
                rightPower = 0;
            }
            controller.setRumble(RumbleType.kLeftRumble, leftPower);
            controller.setRumble(RumbleType.kRightRumble, rightPower);
        }
    }

    private static final Timer  RUMBLE_TIMER   = new Timer("Xbox_Rumble");
    private static final double AXIS_THRESHOLD = 0.5;

    private long   leftTimeout;
    private long   rightTimeout;
    private double leftPower;
    private double rightPower;
    private double deadBand;
    private double axisThreshold;

    public final Trigger aButton          = new Trigger(this::getAButton);
    public final Trigger bButton          = new Trigger(this::getBButton);
    public final Trigger xButton          = new Trigger(this::getXButton);
    public final Trigger yButton          = new Trigger(this::getYButton);
    public final Trigger startButton      = new Trigger(this::getStartButton);
    public final Trigger backButton       = new Trigger(this::getBackButton);
    public final Trigger leftStickButton  = new Trigger(this::getLeftStickButton);
    public final Trigger rightStickButton = new Trigger(this::getRightStickButton);
    public final Trigger leftBumper       = new Trigger(this::getLeftBumper);
    public final Trigger rightBumper      = new Trigger(this::getRightBumper);

    public final Trigger leftTrigger    = new Trigger(() -> getLeftTrigger() >= axisThreshold);
    public final Trigger rightTrigger   = new Trigger(() -> getRightTrigger() >= axisThreshold);
    public final Trigger leftStickXPos  = new Trigger(() -> getLeftStickX() >= axisThreshold);
    public final Trigger leftStickYPos  = new Trigger(() -> getLeftStickY() >= axisThreshold);
    public final Trigger rightStickXPos = new Trigger(() -> getRightStickX() >= axisThreshold);
    public final Trigger rightStickYPos = new Trigger(() -> getRightStickY() >= axisThreshold);
    public final Trigger leftStickXNeg  = new Trigger(() -> getLeftStickX() <= -axisThreshold);
    public final Trigger leftStickYNeg  = new Trigger(() -> getLeftStickY() <= -axisThreshold);
    public final Trigger rightStickXNeg = new Trigger(() -> getRightStickX() <= -axisThreshold);
    public final Trigger rightStickYNeg = new Trigger(() -> getRightStickY() <= -axisThreshold);

    private final XboxController controller;

    public DTXboxController(int port) {
        this.controller = new XboxController(port);
        this.deadBand = 0D;
        this.axisThreshold = AXIS_THRESHOLD;
        RUMBLE_TIMER.scheduleAtFixedRate(new RumbleTask(), 10, 20);
    }

    public int getDpad() {
        return this.controller.getPOV();
    }

    /**
     * Sets the rumble on the controller
     *
     * @param duration
     *        Time in seconds for the rumble to last
     */
    public void startRumble(double duration) {
        startRumble(duration, 1);
    }

    /**
     * Sets the rumble on the controller
     *
     * @param duration
     *        Time in seconds for the rumble to last
     * @param side
     *        What side the ruble on <code>LEFT<code>, <code>RIGHT<code>,
     *        <code>BOTH<code>
     */
    public void startRumble(double duration, RumbleSide side) {
        startRumble(duration, 1, side);
    }

    /**
     * Sets the rumble on the controller
     *
     * @param duration
     *        Time in seconds for the rumble to last
     * @param power
     *        Strength of rumble. Values range from 0-1
     */
    public void startRumble(double duration, double power) {
        startRumble(duration, power, RumbleSide.BOTH);
    }

    /**
     * Sets the rumble on the controller
     *
     * @param duration
     *        Time in seconds for the rumble to last
     * @param power
     *        Strength of rumble. Values range from 0-1
     * @param side
     *        What side the ruble on <code>LEFT<code>, <code>RIGHT<code>,
     *        <code>BOTH<code>
     */
    public void startRumble(double duration, double power, RumbleSide side) {
        long timeout = (long) (1_000 * duration) + System.currentTimeMillis();
        if (side.isLeft) {
            this.leftPower = power;
            this.leftTimeout = timeout;
        }
        if (side.isRight) {
            this.rightPower = power;
            this.rightTimeout = timeout;
        }
    }

    /**
     * Stops the rumble on both sides
     */
    public void stopRumble() {
        stopRumble(RumbleSide.BOTH);
    }

    /**
     * Stops the rumble on a certain side
     *
     * @param side
     *        What side to stop the ruble on <code>LEFT<code>,
     *        <code>RIGHT<code>, <code>BOTH<code>
     */
    public void stopRumble(RumbleSide side) {
        if (side.isLeft) {
            this.leftTimeout = Long.MIN_VALUE;
        }
        if (side.isRight) {
            this.rightTimeout = Long.MIN_VALUE;
        }
    }

    public boolean getAButton() {
        return this.controller.getAButton();
    }

    public boolean getBButton() {
        return this.controller.getBButton();
    }

    public boolean getXButton() {
        return this.controller.getXButton();
    }

    public boolean getYButton() {
        return this.controller.getYButton();
    }

    public boolean getStartButton() {
        return this.controller.getStartButton();
    }

    public boolean getBackButton() {
        return this.controller.getBackButton();
    }

    public boolean getLeftStickButton() {
        return this.controller.getLeftStickButton();
    }

    public boolean getRightStickButton() {
        return this.controller.getRightStickButton();
    }

    public boolean getLeftBumper() {
        return this.controller.getLeftBumper();
    }

    public boolean getRightBumper() {
        return this.controller.getRightBumper();
    }

    public boolean getAButtonPressed() {
        return this.controller.getAButtonPressed();
    }

    public boolean getBButtonPressed() {
        return this.controller.getBButtonPressed();
    }

    public boolean getXButtonPressed() {
        return this.controller.getXButtonPressed();
    }

    public boolean getYButtonPressed() {
        return this.controller.getYButtonPressed();
    }

    public boolean getStartButtonPressed() {
        return this.controller.getStartButtonPressed();
    }

    public boolean getBackButtonPressed() {
        return this.controller.getBackButtonPressed();
    }

    public boolean getLeftStickButtonPressed() {
        return this.controller.getLeftStickButtonPressed();
    }

    public boolean getRightStickButtonPressed() {
        return this.controller.getRightStickButtonPressed();
    }

    public boolean getLeftBumperPressed() {
        return this.controller.getLeftBumperPressed();
    }

    public boolean getRightBumperPressed() {
        return this.controller.getRightBumperPressed();
    }

    public boolean getAButtonReleased() {
        return this.controller.getAButtonReleased();
    }

    public boolean getBButtonReleased() {
        return this.controller.getBButtonReleased();
    }

    public boolean getXButtonReleased() {
        return this.controller.getXButtonReleased();
    }

    public boolean getYButtonReleased() {
        return this.controller.getYButtonReleased();
    }

    public boolean getStartButtonReleased() {
        return this.controller.getStartButtonReleased();
    }

    public boolean getBackButtonReleased() {
        return this.controller.getBackButtonReleased();
    }

    public boolean getLeftStickButtonReleased() {
        return this.controller.getLeftStickButtonReleased();
    }

    public boolean getRightStickButtonReleased() {
        return this.controller.getRightStickButtonReleased();
    }

    public boolean getLeftBumperReleased() {
        return this.controller.getLeftBumperReleased();
    }

    public boolean getRightBumperReleased() {
        return this.controller.getRightBumperReleased();
    }

    public double getLeftStickX() {
        return deadBand(this.controller.getLeftX());
    }

    public double getLeftStickY() {
        return deadBand(-this.controller.getLeftY());
    }

    public double getRightStickX() {
        return deadBand(this.controller.getRightX());
    }

    public double getRightStickY() {
        return deadBand(-this.controller.getRightY());
    }

    public double getLeftTrigger() {
        return deadBand(this.controller.getLeftTriggerAxis());
    }

    public double getRightTrigger() {
        return deadBand(this.controller.getRightTriggerAxis());
    }

    public double getLeftStickXSquared() {
        return squareKeepSign(getLeftStickX());
    }

    public double getLeftStickYSquared() {
        return squareKeepSign(getLeftStickY());
    }

    public double getRightStickXSquared() {
        return squareKeepSign(getRightStickX());
    }

    public double getRightStickYSquared() {
        return squareKeepSign(getRightStickY());
    }

    public double getLeftTriggerSquared() {
        return squareKeepSign(getLeftTrigger());
    }

    public double getRightTriggerSquared() {
        return squareKeepSign(getRightTrigger());
    }

    public void setDeadBand(double deadBand) {
        this.deadBand = deadBand;
    }

    public void setAxisThreshold(double threshold) {
        this.axisThreshold = threshold;
    }

    private double deadBand(double d) {
        if (this.deadBand < 1e-6) {
            // Not set, "fast path"
            return d;
        } else if (Math.abs(d) < this.deadBand) {
            return 0D;
        } else if (d < 0D) {
            return (d + this.deadBand) / (1D - this.deadBand);
        } else {
            return (d - this.deadBand) / (1D - this.deadBand);
        }
    }

    private static double squareKeepSign(double d) {
        return Math.copySign(d * d, d);
    }
}
