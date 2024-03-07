// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.Colors;
import frc.robot.subsystems.leds.LEDConstants.Configs;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private boolean shouldAnimate;

  public LEDSubsystem() {
    m_led = new AddressableLED(Configs.PWM_PORT);

    m_ledBuffer = new AddressableLEDBuffer(Configs.LED_COUNT);
    m_led.setLength(m_ledBuffer.getLength());

    shouldAnimate = false;
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
    int extra = m_ledBuffer.getLength() % 3;
    boolean isBlue = true;
    for (int i = 0; i < m_ledBuffer.getLength() - extra; i += 3) {
      if (i % 2 == 0) {
        isBlue = true;
        m_ledBuffer.setHSV(i, Colors.BLUE, 100, 100);
        m_ledBuffer.setHSV(i + 1, Colors.BLUE, 100, 100);
        m_ledBuffer.setHSV(i + 2, Colors.BLUE, 100, 100);
      } else {
        isBlue = false;
        m_ledBuffer.setHSV(i, Colors.PURPLE, 100, 100);
        m_ledBuffer.setHSV(i + 1, Colors.PURPLE, 100, 100);
        m_ledBuffer.setHSV(i + 2, Colors.PURPLE, 100, 100);
      }
    }

    for (int i = m_ledBuffer.getLength() - 1; i >= m_ledBuffer.getLength() - 1 - extra; i--) {
      if (isBlue) {
        m_ledBuffer.setHSV(i, Colors.PURPLE, 100, 100);
      } else {
        m_ledBuffer.setHSV(i, Colors.BLUE, 100, 100);
      }
    }
    setBuffer();
  }

  public void alternateRedGreenLED(double val1, double val2, boolean isAtGoal) {
    alternateRedGreenLED(val1, val2);

    if (isAtGoal) everythingReady();
  }

  public void alternateRedGreenLED(double val1, double val2) {}

  public void blueBlackWhiteGradient() {}

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

  public int[] hexToHSV(String hexColor) {
    // Convert hex to RGB
    hexColor = hexColor.toUpperCase();
    int r = Integer.valueOf(hexColor.substring(1, 3), 16);
    int g = Integer.valueOf(hexColor.substring(3, 5), 16);
    int b = Integer.valueOf(hexColor.substring(5, 7), 16);

    // Convert RGB to HSV
    float[] hsv = new float[3];
    java.awt.Color.RGBtoHSB(r, g, b, hsv);

    // Adjust hue to be in the range of 0-180
    hsv[0] *= 180;
    hsv[1] *= 100;
    hsv[2] *= 100;

    // Convert float array to double for the return value
    int[] adjustedHSV = new int[hsv.length];
    for (int i = 0; i < hsv.length; i++) {
      adjustedHSV[i] = (int) hsv[i];
    }

    return adjustedHSV;
  }

  // private void wait(int milliseconds) {
  //   double initialTime = Timer.getFPGATimestamp();

  //   while (Timer.getFPGATimestamp() - initialTime < milliseconds / 1000.0) {
  //     // Do nothing
  //   }
  // }

  // private Command setLEDCommand(int index, int hue, int saturation, int value) {
  //   return new InstantCommand(
  //       () -> {
  //         m_ledBuffer.setHSV(index, hue, saturation, value);
  //         setBuffer();
  //       });
  // }

  @Override
  public void periodic() {
    if (shouldAnimate) {
      Color lastLED = m_ledBuffer.getLED(m_ledBuffer.getLength() - 1);
      for (int i = 1; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, m_ledBuffer.getLED(i - 1));
        setBuffer();
      }
      m_ledBuffer.setLED(0, lastLED);
      setBuffer();
    }
  }
}
