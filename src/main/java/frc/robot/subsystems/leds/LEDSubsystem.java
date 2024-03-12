// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import java.util.ArrayList;
import java.util.PriorityQueue;
import java.util.Queue;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.Configs;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  private ArrayList<Runnable> queuedTasks;
  private double taskTargetTimestamp;
  private double taskDelaySec;
  
  private ArrayList<Runnable> queuedPatterns;
  private double patternTargetTimestamp;
  private double patternDelaySec;

  public LEDSubsystem() {
    led = new AddressableLED(Configs.PWM_PORT);
    ledBuffer = new AddressableLEDBuffer(Configs.LED_COUNT);
    led.setLength(ledBuffer.getLength());

    queuedTasks = new ArrayList<Runnable>();
    taskTargetTimestamp = Timer.getFPGATimestamp();
    taskDelaySec = 0.1;
    
    queuedPatterns = new ArrayList<Runnable>();
    patternTargetTimestamp = Timer.getFPGATimestamp();
    patternDelaySec = 0.0;
  }

  
  private void setSolidBlue() {
  }
  
  private void setSolidBlueGradient() {
    
  }
  
  private void clearAnimations() {
    queuedPatterns.clear();
    queuedTasks.clear();
  }
  
  private void setBuffer() {
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    if(queuedTasks.size() == 0){
      if(queuedPatterns.size() == 0) return;
      
      queuedPatterns.

      queuedPatterns.remove(0).run();
    }

    if(ledTasks.size() >= 1 && Timer.getFPGATimestamp() >= targetTimestamp) {
      ledTasks.remove(0).run();
      targetTimestamp = Timer.getFPGATimestamp() + updateDelay;
    }
  }
}
