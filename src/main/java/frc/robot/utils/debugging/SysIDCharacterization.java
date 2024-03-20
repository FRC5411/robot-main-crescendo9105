package frc.robot.utils.debugging;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

public class SysIDCharacterization {
  public static Command runShooterSysIDTests(Consumer<Double> voltageSetter, Subsystem subsystem) {
    SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Seconds.of(1)),
                Units.Volts.of(4),
                Units.Seconds.of(15),
                (state) -> sysIDCTREStateLogger(state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> voltageSetter.accept(voltage.magnitude()), null, subsystem));

    return new SequentialCommandGroup(
        startCTRELoggingRoutine(),
        Commands.waitSeconds(5),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(5),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(5),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(5),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(5),
        stopCTRELoggingRoutine());
    // For rev logs extract using wpilib's data log tool:
    // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog-download.html
    // For talon logs extract using phoenix tuner x:
    // https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tools/log-extractor.html
  }

  private static Command startCTRELoggingRoutine() {
    return Commands.runOnce(() -> SignalLogger.start());
  }

  private static void sysIDCTREStateLogger(String state) {
    SignalLogger.writeString("Shooter/sysIDTestState", state);
  }

  private static Command stopCTRELoggingRoutine() {
    return Commands.runOnce(() -> SignalLogger.stop());
  }

  public static Command runDriveSysIDTests(Consumer<Double> voltageSetter, Subsystem subsystem) {
    SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Seconds.of(1)),
                Units.Volts.of(4),
                Units.Seconds.of(15),
                (state) -> sysIDREVStateLogger(state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> voltageSetter.accept(voltage.magnitude()), null, subsystem));

    return new SequentialCommandGroup(
        Commands.waitSeconds(5),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(5),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(5),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(5),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(5));
    // For rev logs extract using wpilib's data log tool:
    // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog-download.html
    // For talon logs extract using phoenix tuner x:
    // https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tools/log-extractor.html
  }


  private static void sysIDREVStateLogger(String state) {
    Logger.recordOutput("Shooter/sysIDTestState", state);
  }
}