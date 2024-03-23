// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.Configs;
import frc.robot.subsystems.leds.LEDConstants.RGB;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  public LEDSubsystem() {
    led = new AddressableLED(Configs.PWM_PORT);
    ledBuffer = new AddressableLEDBuffer(Configs.LED_COUNT);
    led.setLength(ledBuffer.getLength());
  }

  public void setReadyColor() {
    setSolidColor(RGB.GREEN[0], RGB.GREEN[1], RGB.GREEN[2]);
  }

  public void setDefaultColor() {
    setSolidColor(RGB.DARK_BLUE[0], RGB.DARK_BLUE[1], RGB.DARK_BLUE[2]);
  }

  public void setHasPiece() {
    setSolidColor(RGB.ORANGE[0], RGB.ORANGE[1], RGB.ORANGE[2]);
  }

  public void turnOffLEDs() {
    setSolidColor(0, 0, 0);
  }

  public void setSolidColor(int r, int g, int b) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
    setBuffer();
  }

  private void setBuffer() {
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {}
}
