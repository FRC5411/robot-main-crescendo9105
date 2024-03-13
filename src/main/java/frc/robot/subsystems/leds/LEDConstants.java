// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

public class LEDConstants {
  public class Hues {
    public static final int LIGHT_BLUE = 200 / 2;
    public static final int DARK_BLUE = 240 / 2;
    public static final int GREEN = 130 / 2;
    public static final int RED = 0;
  }

  public class Configs {
    public static final int LED_COUNT = 82;
    public static final int PWM_PORT = 0;
    public static final int[] SHOOTER_LEDS = {0, 27};
    public static final int[] RIGHT_ARM_LEDS = {28, 55};
    public static final int[] LEFT_ARM_LEDS = {56, 82};
  }
}
