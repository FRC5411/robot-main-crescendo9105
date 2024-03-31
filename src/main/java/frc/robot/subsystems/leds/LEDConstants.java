// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

public class LEDConstants {
  public class RGB {
    public static final int[] DARK_BLUE = {0, 8, 255};
    public static final int[] GREEN = {32, 240, 0};
    public static final int[] ORANGE = {255, 85, 0};
  }

  public class Configs {
    public static final int LED_COUNT = 82;
    public static final int PWM_PORT = 0;
    public static final int[] SHOOTER_LEDS = {0, 24};
    public static final int[] RIGHT_ARM_LEDS = {25, 51};
    public static final int[] LEFT_ARM_LEDS = {52, 81};
  }
}
