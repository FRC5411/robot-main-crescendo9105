// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.Configs;
import java.util.ArrayList;
//import java.util.Arrays;
import java.util.Collections;
//import java.util.List;
//import java.util.function.IntPredicate;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  private ArrayList<Runnable> queuedTasks;
  private double taskTargetTimestamp;
  private double taskDelaySec;

  private ArrayList<Runnable> queuedPatterns;
  private double patternTargetTimestamp;
  private double patternDelaySec;

  private boolean shouldAnimate;
  private double animateTargetTimestamp;
  private double animateDelaySec;

  private boolean direction;
  private boolean tasksJustRanOut;

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

    animateTargetTimestamp = Timer.getFPGATimestamp();
    animateDelaySec = 1;
    shouldAnimate = false;

    direction = true;
    tasksJustRanOut = false;
  }

  public void setSolidBlue() {
    setSolidHSV(228/2, 100, 100);
  }

  private void setSolidHSV(int h, int s, int v) {
    shouldAnimate = false;
    queuedPatterns.add(() -> {
      taskDelaySec = 0.2;
      for(int i = 0; i < ledBuffer.getLength(); i++) {
        final int index = i;
        queuedTasks.add(() -> ledBuffer.setHSV(index, h, s, v));
      }
    });
  }  

  public void dynamicPingPong(int affectedColor, int otherColor){ 
     ArrayList<Integer> affectedLeds = new ArrayList<>(); 
     affectedLeds.add(0);
     boolean right = true;   
     direction = false;
     while (affectedLeds.size() < ledBuffer.getLength()){   
          queuedPatterns.add(() -> { 
           taskDelaySec = 0.2;
           for (int idx = 0; idx < ledBuffer.getLength(); idx ++){  
               final int index = idx; 
               if (affectedLeds.contains(idx)){ 
                  queuedTasks.add(() -> ledBuffer.setHSV(index, affectedColor,255,255));
               } else { 
                  queuedTasks.add(() -> ledBuffer.setHSV(index, otherColor, 255,255));
              }
            } 
          }); 
          setBuffer();
          if (right){ 
            if (Collections.max(affectedLeds) == ledBuffer.getLength()-1){  
              affectedLeds.add(Collections.min(affectedLeds) - 1);
              right = false;
            } 
            affectedLeds.forEach((number) -> { 
              number += 1;
            }); 
          } else { 
            if (Collections.min(affectedLeds) == 0){  
              affectedLeds.add(Collections.max(affectedLeds) + 1);
              right = true;
            } 
            affectedLeds.forEach((number) -> { 
              number -= 1;
            }); 
          }
     } 
     direction = true;
  } 

  

  private void setBuffer() {
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    if (shouldAnimate) {
      if(Timer.getFPGATimestamp() >= animateTargetTimestamp) {
        // Animate a single frameshift.
        if (direction){
          Color lastLED = ledBuffer.getLED(ledBuffer.getLength() - 1);
          for (int i = ledBuffer.getLength() - 1; i > 0; i--)
            ledBuffer.setLED(i, ledBuffer.getLED(i - 1));
          ledBuffer.setLED(0, lastLED);
        } else  {
          Color firstLED = ledBuffer.getLED(0);
          for (int i = 0; i < ledBuffer.getLength(); i++)
            ledBuffer.setLED(i, ledBuffer.getLED(i + 1));
          ledBuffer.setLED(ledBuffer.getLength() - 1, firstLED);
        }
        setBuffer();
        animateTargetTimestamp = Timer.getFPGATimestamp() + animateDelaySec;
      } 
    } else {
      if (queuedTasks.size() == 0) {
        if (queuedPatterns.size() > 0) {
          if (!tasksJustRanOut) {
            patternTargetTimestamp = Timer.getFPGATimestamp() + patternDelaySec;
            tasksJustRanOut = true;
          }

          if (Timer.getFPGATimestamp() >= patternTargetTimestamp) {
            queuedPatterns.remove(0).run(); // The pattern should populate the tasks arraylist
            tasksJustRanOut = false;
          }
        }
      } else if (Timer.getFPGATimestamp() >= taskTargetTimestamp) {
          queuedTasks.remove(0).run();
          taskTargetTimestamp = Timer.getFPGATimestamp() + taskDelaySec;
      }
    }
  }
}
