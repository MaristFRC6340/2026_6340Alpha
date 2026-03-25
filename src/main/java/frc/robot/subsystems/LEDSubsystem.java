package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    
    AddressableLED strip;
    AddressableLEDBuffer buffer;

    public LEDSubsystem() {
        strip = new AddressableLED(9);
        buffer = new AddressableLEDBuffer(100);
        strip.setLength(buffer.getLength());

        strip.setData(buffer);
        strip.start();
        setDefaultCommand(setPattern(LEDPattern.solid(Color.kGreen)));
    }

    @Override
    public void periodic() {
        strip.setData(buffer);
        
        setPattern(LEDPattern.solid(Color.kMagenta)).schedule();
    }

    public Command setPattern(LEDPattern pattern) {
        return this.run(() -> pattern.applyTo(buffer));
    }

    public Command setPattern(Runnable setBuffer) {
        return this.run(() -> setBuffer.run());
    }
}
