// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int PWM_PORT = 9;
  private int LED_COUNT = 60;

  public LEDSubsystem() {
    m_led = new AddressableLED(PWM_PORT);

    m_ledBuffer = new AddressableLEDBuffer(LED_COUNT);
    m_led.setLength(m_ledBuffer.getLength());
  }

  public void rainbowLED() {
    int m_rainbowFirstPixelHue = 0;

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }

    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;

    setBuffer();
  }

  /*
   * @param value A value between 0 and 1 that represents the red to green gradient.
   */
  public void redToGreenLED(double value) {
    // Clamp value between 0 and 1.
    if (value < 0) value = 0;
    if (value > 1) value = 1;
    
    // Hue value for red in HSV is 0 degrees and for green is 120 degrees.
    // Since we want a gradient between red and green, we map the input value to this range.
    // Have to divide by 2 because WPILib hsv range is from 0-180.
    int hue = ((int) Math.round(value * 120)) / 2; // Map the value to the 0-120 degree range.

    setEveryLED(hue, 100, 100);
  }

  /*
   *  Set the LED strip to display a message in morse code.
   * @param message The message to be displayed in morse code with - and . characters. For example, 'synth' would be "... -.-- -. - ...."
   */
  // public void wordToMorseCodeLED(String message) {

  //   String[] letters = message.split(" ");

  //   int ledsNeeded = 0;

  //   for (String letter : letters) {
  //     ledsNeeded += ;
  //   }

  //   for (String letter : letters) {
  //     for (int i = 0; i < letter.length(); i++) {
  //       char character = letter.charAt(i);
  //       if (character == '.') {
          
  //       } else if (character == '-') {
          
  //       } else if (character == ' ') {
          
  //       } else {
  //         throw new IllegalArgumentException("wordToMorseCodeLED() parameter can only contain . or - characters.");
  //       }
  //     }
  //   }
  // }

  private void setEveryLED(int hue, int saturation, int value) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, hue, saturation, value);
    }
    setBuffer();
  }

  private void setBuffer() {
    m_led.setData(m_ledBuffer);
    m_led.start();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
