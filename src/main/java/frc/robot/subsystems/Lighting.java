package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {
    private static final Color[] colors = { Color.purpleColor, Color.yellowColor, Color.blueColor,
            Color.redColor };

    private final int length;
    private int       clock;

    private AddressableLEDBuffer buffer;
    private AddressableLED       ledStrip;

    public Lighting(int portNum, int length) {
        this.length = length;

        buffer = new AddressableLEDBuffer(length);
        ledStrip = new AddressableLED(portNum);
        ledStrip.setLength(length);
        ledStrip.start();

    }

    public void setColor(Color color) {
        for (int i = 0; i < length; i++) {
            setColor(i, color);
        }
    }

    public void setColor(int index, Color color) {
        buffer.setRGB(index, color.red, color.green, color.blue);
    }

    @Override
    public void periodic() {
        clock = (clock + 1) & 255;
        setColor(colors[clock >> 6]);
        ledStrip.setData(buffer);
    }
}
