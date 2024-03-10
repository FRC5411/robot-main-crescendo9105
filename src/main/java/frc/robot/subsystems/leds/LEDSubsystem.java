// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.leds.LEDConstants.Configs;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private boolean shouldAnimate;
  private int curLED;

  public LEDSubsystem() {
    led = new AddressableLED(Configs.PWM_PORT);

    ledBuffer = new AddressableLEDBuffer(Configs.LED_COUNT);
    led.setLength(ledBuffer.getLength());
    curLED = 0;
  }

  private void setBuffer() {
    led.setData(ledBuffer);
    led.start();
  }

  private Command setLEDCommand(int hue, int saturation, int value) {
    return new InstantCommand(
        () -> {
          ledBuffer.setHSV(curLED, hue, saturation, value);
          setBuffer();
        });
  }

  public Command incrementLED() {
    return new InstantCommand(
        () -> {
          if (curLED < ledBuffer.getLength()) curLED++;
        });
  }

  // Average object oriented programming moment
  public Command alternateLED() {
    curLED = 0;
    return new SequentialCommandGroup(
            setLEDCommand(60, 255, 255),
            incrementLED(),
            new WaitCommand(2),
            setLEDCommand(120, 255, 255),
            incrementLED())
        .repeatedly()
        .until(() -> curLED == ledBuffer.getLength() - 1);
  }

  /*
   * @param val A value between 0 and 1 that represents the red to green gradient, 0=red, 1=green.
   */
  public Command redGreenGradientCommand(double val) {
    curLED = 0;
    // Clamp value between 0 and 1.
    if (val < 0) val = 0;
    if (val > 1) val = 1;

    // Hue value for red in HSV is 0 degrees and for green is 120 degrees.
    // Since we want a gradient between red and green, we map the input value to this range.
    // Have to divide by 2 because WPILib hsv range is from 0-180.
    int hue = ((int) Math.round(val * 120)) / 2;

    return setEveryLEDCommand(hue, 255, 255);
  }

  private Command setEveryLEDCommand(int hue, int saturation, int value) {
    return new InstantCommand(
        () -> {
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, hue, saturation, value);
          }
          setBuffer();
        });
  }

  @Override
  public void periodic() {}
}
