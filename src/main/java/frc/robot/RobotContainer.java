// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import javax.swing.text.html.Option;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.feeder.FeederIOReal;
import frc.robot.Subsystems.feeder.FeederIOSim;
import frc.robot.Subsystems.feeder.FeederSubsystem;
import frc.robot.Subsystems.feeder.RoutingSim;
import frc.robot.Subsystems.feeder.beambreak.BeamBreakIO;
import frc.robot.Subsystems.feeder.beambreak.BeamBreakIOReal;
import frc.robot.Subsystems.feeder.beambreak.BeamBreakIOSim;
import frc.robot.Subsystems.shooter.*;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  

  BeamBreakIO beamBreakIO = null;

  ShooterSubsystem shooterSubsystem = null;
  FeederSubsystem feederSubsystem = null;


  public RobotContainer() {

    if (Robot.isReal()) {
      beamBreakIO = new BeamBreakIOReal();
      shooterSubsystem = new ShooterSubsystem(
        // Make sure these inverts are correct
        new FlywheelIOReal(Constants.FLYWHEEL_KRAKEN_ID1, InvertedValue.Clockwise_Positive), 
        new FlywheelIOReal(Constants.FLYWHEEL_KRAKEN_ID2, InvertedValue.CounterClockwise_Positive), 
        new PivotIOReal());
      feederSubsystem = new FeederSubsystem(new FeederIOReal(), beamBreakIO);
    } else {
      BeamBreakIOSim beamBreakIO = new BeamBreakIOSim();
      RoutingSim.getInstance().setBeamBreakIOSim(beamBreakIO);
      shooterSubsystem = new ShooterSubsystem(new FlywheelIOSim(), new FlywheelIOSim(), new PivotIOSim());
      feederSubsystem = new FeederSubsystem(new FeederIOSim(), beamBreakIO);
    }

    feederSubsystem.setDefaultCommand(feederSubsystem.setVelocityCommand(0));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.runShooterCommand(new Rotation2d(0), () -> 0, () -> 0));
    configureBindings();

  }

  private void configureBindings() {
    controller.x().whileTrue(Commands.deadline(
      // Hopefully runs flywheels for 1 sec, then turns them off
      feederSubsystem.setVelocityCommand(80).withTimeout(1),
      // Tune values
      shooterSubsystem.runShooterCommand(new Rotation2d(30.0), () -> -80, () -> -80),
      // Reset sim position
      Commands.runOnce(() -> {
        RoutingSim.getInstance().setNotePos(Optional.empty());
      })
    ));
    controller.b().whileTrue(shooterSubsystem.setPivotAngleCommand(new Rotation2d(50)));
    controller.y().whileTrue(feederSubsystem.indexCommand());
    controller.a().whileTrue(
      Commands.runOnce(() -> {
        RoutingSim.getInstance().setNotePos(RoutingSim.getInstance().getNotePos().isEmpty() ? Optional.of(0.0) : RoutingSim.getInstance().getNotePos());
      }).andThen(feederSubsystem.setVelocityCommand(10))
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


}
