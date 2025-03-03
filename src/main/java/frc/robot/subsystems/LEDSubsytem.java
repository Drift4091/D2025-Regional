package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsytem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    public LEDSubsytem(int port, int length){
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    public void setColor(int r, int g, int b){
        for(int i = 0; i < buffer.getLength(); i++){
            buffer.setRGB(i, r, g, b);
        }
        led.setData(buffer);
    }
}
