// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.Configs;
import frc.robot.subsystems.leds.LEDConstants.Hues;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  public LEDSubsystem() {
    led = new AddressableLED(Configs.PWM_PORT);
    ledBuffer = new AddressableLEDBuffer(Configs.LED_COUNT);
    led.setLength(ledBuffer.getLength());
  
    setDefaultColor();
  }

  public void setReadyColor() {
    setSolidColor(Hues.LIGHT_BLUE, 255, 255);
  }

  public void setDefaultColor() {
    setSolidColor(Hues.DARK_BLUE, 255, 255);
  }

  /*
   * This class is used to set the color of the LED strip to red, green, or anything in between
   * @param percentage 0-100, 0 being red, 100 being green
   */
  private int getRedGreenHue(int percentage) {
    double percentDecimal = MathUtil.clamp(percentage, 0, 100) / 100.0;
    return (int) ((Hues.GREEN - Hues.RED) * percentDecimal + Hues.RED);
  }

  public void setSolidColor(int h, int s, int v){
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, h, s, v);
    }
    setBuffer();
  }

  private void setBuffer() {
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
  }
}
