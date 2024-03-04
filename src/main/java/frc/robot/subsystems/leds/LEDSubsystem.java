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

  /*
   * Set the LED strip to display a rainbow pattern.
   * rainbowLED() is from WPILib documentation.
   */
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
   * Set the LED strip to display a gradient from red to green.
   * @param value A value between 0 and 1 that represents the red to green gradient.
   */
  public void solidRedToGreenLED(double value) {
    // Clamp value between 0 and 1.
    if (value < 0) value = 0;
    if (value > 1) value = 1;

    // Hue value for red in HSV is 0 degrees and for green is 120 degrees.
    // Since we want a gradient between red and green, we map the input value to this range.
    // Have to divide by 2 because WPILib hsv range is from 0-180.
    int hue = ((int) Math.round(value * 120)) / 2; // Map the value to the 0-120 degree range.

    setEveryLED(hue, 100, 100);
  }

  public void everythingReady() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 2 == 0) m_ledBuffer.setHSV(i, 215 / 2, 100, 100);
      else m_ledBuffer.setHSV(i, 280 / 2, 100, 100);
    }
    setBuffer();
  }

  public void alternateRedGreenLED(double val1, double val2, boolean isAtGoal) {
    alternateRedGreenLED(val1, val2);

    if (isAtGoal) everythingReady();
  }

  public void alternateRedGreenLED(double val1, double val2) {}

  /*
   *  [NOT COMPLETE, DO NOT USE] Set the LED strip to display a message in morse code.
   * @param message The message to be displayed in morse code with - and . characters. For example, 'synth' would be "... -.-- -. - ...."
   */
  public void wordToMorseCodeLED(String message) {
    final int DOT_LENGTH = 1;
    final int DASH_LENGTH = 3;
    final int MORSE_SPACE = 1; // space between each - or . in a letter.
    final int LETTER_SPACE = 3; // space between each letter in morse code
    final int REPEAT_SPACE = 4; // space before the animation repeats
    final int WORD_SPACE = 1; // space before and after each word
    final boolean CHECK_SPACE = true; // checks if there is enough space before repeating

    String[] letters = message.split(" ");

    int ledsNeeded = 0;

    if (CHECK_SPACE) {
      ledsNeeded = WORD_SPACE + (letters.length - 1) * LETTER_SPACE + WORD_SPACE;
      // Make sure each letter is valid. And allocate space for the LED buffer.
      for (String letter : letters) {
        if (letter.length() > 5)
          throw new IllegalArgumentException(
              "wordToMorseCodeLED() invalid morse letter, maximum length 5 for each letter.");
        for (int i = 0; i < letter.length(); i++) {
          char character = letter.charAt(i);

          if (character == '.') ledsNeeded += DOT_LENGTH;
          else if (character == '-') ledsNeeded += DASH_LENGTH;
          else
            throw new IllegalArgumentException(
                "wordToMorseCodeLED() parameter can only contain . or - characters.");
        }
        ledsNeeded += (letter.length() - 1) * MORSE_SPACE;
      }
    }
    int ledsLeft = m_ledBuffer.getLength() - ledsNeeded;

    while (ledsLeft >= 0) {
      for (String letter : letters) {
        for (int i = 0; i < letter.length(); i++) {
          char character = letter.charAt(i);
          if (character == '.') {

          } else if (character == '-') {

          }
        }
      }

      if (CHECK_SPACE) ledsLeft -= REPEAT_SPACE;
    }

    setBuffer();
  }

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
