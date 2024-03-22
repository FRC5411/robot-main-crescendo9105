// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  public static final Robot currentRobot = Robot.SYNTH;
  public static boolean tuningMode = true;
  public static boolean useDebuggingBindings = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum Robot {
    /** NEO Swerve robot */
    NEOSWERVE,
    /** Alphabot constants */
    SYNTH
  }

  public static final Pose3d kSpeaker3DPose =
      (DriverStation.getAlliance().isPresent())
          ? new Pose3d(
              new Translation3d(
                  (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                      ? 0.23
                      : 16.54 - 0.23,
                  8.27 / 2.0 + 1.4478,
                  2.045),
              new Rotation3d())
          : new Pose3d(new Translation3d(0.23, 8.27 / 2 + 1.4478, 2.045), new Rotation3d());
}
