// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.schema;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/** Object to hold the keybindings and preferences for the pilots */
public class PilotProfile {
  private final Constants.Pilot NAME;

  private Map<Constants.Bindings, Trigger> keybindings = new HashMap<>();
  private Map<Constants.Preferences, Object> preferences = new HashMap<>();

  /** Create a new profile */
  public PilotProfile(Constants.Pilot pilotName) {
    NAME = pilotName;
  }

  /** Set a button binding */
  public void setBinding(Constants.Bindings buttonKey, Trigger binding) {
    keybindings.put(buttonKey, binding);
  }

  /** Set a preference (E.g. deadzones) */
  public void setPreference(Constants.Preferences preferenceKey, Object preference) {
    preferences.put(preferenceKey, preference);
  }

  /** Returns the desired keybinding */
  public Trigger getBinding(Constants.Bindings buttonKey) {
    return keybindings.get(buttonKey);
  }

  /** Return sthe desired preference */
  public Object getPreference(Constants.Preferences preferenceKey) {
    return preferences.get(preferenceKey);
  }

  /** Returns the pilot's name */
  public Constants.Pilot getName() {
    return NAME;
  }
}
