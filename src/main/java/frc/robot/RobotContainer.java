// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.feeder.RoutingSim;
import frc.robot.Subsystems.feeder.beambreak.BeamBreakIO;
import frc.robot.Subsystems.feeder.beambreak.BeamBreakIOReal;
import frc.robot.Subsystems.feeder.beambreak.BeamBreakIOSim;
import frc.robot.Subsystems.feeder.feeder.FeederIOReal;
import frc.robot.Subsystems.feeder.feeder.FeederIOSim;
import frc.robot.Subsystems.feeder.feeder.FeederSubsystem;
import frc.robot.Subsystems.shooter.*;
import frc.robot.Subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.Subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.Subsystems.shooter.pivot.PivotIOReal;
import frc.robot.Subsystems.shooter.pivot.PivotIOSim;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  

  BeamBreakIO beamBreakIO = null;

  ShooterSubsystem shooterSubsystem = null;
  FeederSubsystem feederSubsystem = null;

  private static double batteryVoltage;

  private static boolean hasPassedBeamBreak = false;


  public RobotContainer() {

    if (Robot.isReal()) {
      beamBreakIO = new BeamBreakIOReal();
      shooterSubsystem = new ShooterSubsystem(
        // Make sure these inverts are correct
        new FlywheelIOReal(Constants.FLYWHEEL_TALON_ID1, InvertedValue.Clockwise_Positive), 
        new FlywheelIOReal(Constants.FLYWHEEL_TALON_ID2, InvertedValue.CounterClockwise_Positive), 
        new PivotIOReal());
      feederSubsystem = new FeederSubsystem(new FeederIOReal(), beamBreakIO);
    } else {
      BeamBreakIOSim beamBreakIO = new BeamBreakIOSim();
      RoutingSim.getInstance().setBeamBreakIOSim(beamBreakIO);
      shooterSubsystem = new ShooterSubsystem(new FlywheelIOSim(1), new FlywheelIOSim(2), new PivotIOSim());
      feederSubsystem = new FeederSubsystem(new FeederIOSim(), beamBreakIO);
    }

    feederSubsystem.setDefaultCommand(feederSubsystem.setVelocityCommand(0));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.runShooterCommand(new Rotation2d(0), () -> 0, () -> 0));
    configureBindings();

  }

  private void configureBindings() {
    new Trigger(() -> feederSubsystem.getBeamBreakIO().getSecondBeamBreak()).onTrue(Commands.runOnce(() -> {hasPassedBeamBreak = true;}));
    controller.x().onTrue(Commands.deadline(
            shooterSubsystem.runShooterCommand(Rotation2d.fromDegrees(30), () -> 80, () -> 80),
      // Spins up flywheels for 1 sec then shoots note
            Commands.sequence(
                    Commands.waitSeconds(1),
                    feederSubsystem.setVelocityCommand(50)
                            .raceWith(Commands.waitUntil(() -> (!(feederSubsystem.getBeamBreakIO().getSecondBeamBreak()) && !(feederSubsystem.getBeamBreakIO().getFirstBeamBreak()) && hasPassedBeamBreak)
                            )).andThen(() -> hasPassedBeamBreak = false)
            )

    )
    .andThen(Commands.deadline(
            shooterSubsystem.setFlywheelVelocityCommand(() -> 0.0, () -> 0.0),
            feederSubsystem.setVelocityCommand(0)
    )));
    controller.b().whileTrue(shooterSubsystem.setPivotAngleCommand(new Rotation2d(0.4)));
    //controller.b().whileTrue(shooterSubsystem.setFlywheelVelocityCommand(() -> 0, () -> 0));
    //controller.y().onTrue(feederSubsystem.indexCommand());
    controller.y().onTrue(shooterSubsystem.setFlywheelVelocityCommand(() -> 0, () -> 0).alongWith(Commands.runOnce(() -> hasPassedBeamBreak = false)));
    controller.a().onTrue(Commands.sequence(
      Commands.runOnce(() -> {
        if (Robot.isSimulation())
          RoutingSim.getInstance().setNotePos(RoutingSim.getInstance().getNotePos().isEmpty() ? Optional.of(0.450 + Units.inchesToMeters(14)) : RoutingSim.getInstance().getNotePos());
      }),
      Commands.parallel(
         shooterSubsystem.setFlywheelVelocityCommand(() -> -5.0, () -> -5.0).until(() -> feederSubsystem.getBeamBreakIO().getFirstBeamBreak() && !feederSubsystem.getBeamBreakIO().getSecondBeamBreak()).andThen(shooterSubsystem.setFlywheelVelocityCommand(() -> 0, () -> 0)),
         feederSubsystem.indexCommandWithVelocity(30.0))
      )

    );
    //controller.a().whileTrue(feederSubsystem.setVelocityCommand(5.0));
    //controller.a().whileTrue(shooterSubsystem.setFlywheelVelocityCommand(() -> 10, () -> 20));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodic() {
      RobotContainer.batteryVoltage = MathUtil.clamp(BatterySim.calculateDefaultBatteryLoadedVoltage(
        shooterSubsystem.getFlywheelLeftCurrentAmps(),
        shooterSubsystem.getFlywheelRightCurrentAmps(),
        shooterSubsystem.getPivotMotorCurrentAmps(),
        feederSubsystem.getFeederMotorCurrentAmps()
      ), 0, 12);

      Logger.recordOutput("Battery Voltage", RobotContainer.getBatteryVoltage());
      Logger.recordOutput("HasPassedBeamBreak", hasPassedBeamBreak);
  }


  public static double getBatteryVoltage() {
    return RobotContainer.batteryVoltage;
  }


}
