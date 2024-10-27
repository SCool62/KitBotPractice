// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder.feeder;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems.feeder.RoutingSim;
import frc.robot.Subsystems.feeder.beambreak.BeamBreakIO;
import frc.robot.Subsystems.feeder.beambreak.BeamBreakIOInputsAutoLogged;
import frc.robot.Subsystems.feeder.feeder.FeederIO.FeederIOInputs;

/** Add your docs here. */
public class FeederSubsystem extends SubsystemBase {
    public static final double INDEXING_VOLTAGE = 4.0;
    public static final double INDEXING_VELOCITY = 40.0;

    private FeederIO feederIO;
    private BeamBreakIO beamBreakIO;

    private BeamBreakIOInputsAutoLogged beamBreakIOInputs;
    private FeederIOInputsAutoLogged feederIOInputs;

    private double velocityTarget = 0.0;

    public FeederSubsystem(FeederIO feederIO, BeamBreakIO beamBreakIO) {
        this.feederIO = feederIO;
        this.beamBreakIO = beamBreakIO;

        this.beamBreakIOInputs = new BeamBreakIOInputsAutoLogged();
        this.feederIOInputs = new FeederIOInputsAutoLogged();

    }

    @Override
    public void periodic() {
        beamBreakIO.updateInputs(beamBreakIOInputs);
        feederIO.updateInputs(feederIOInputs);

        Logger.recordOutput("FeederVTarget", velocityTarget);
        Logger.processInputs("Feeder", feederIOInputs);
        Logger.processInputs("BeamBreak", beamBreakIOInputs);
        Logger.recordOutput("FeederVTarget", velocityTarget);
    }

    public Command setVelocityCommand(double velocity) {
        return this.run(() -> {
            velocityTarget = velocity;
            feederIO.setVelocity(velocity);
        });
    }

    public Command setVoltageCommand(double voltage) {
        return this.run(() -> feederIO.setVoltage(voltage));
    }

    public Command indexCommand() {
        return this.run(() -> {
            if (Robot.isSimulation() && RoutingSim.getInstance().getNotePos().isEmpty()) {
                System.out.println("No note loaded");
                return;
            }

            if (beamBreakIOInputs.secondBeamBreak) {
                // Move ring backward
                velocityTarget = -INDEXING_VELOCITY / 4;
                feederIO.setVelocity(-INDEXING_VELOCITY / 4);
            } else if (beamBreakIOInputs.firstBeamBreak) {
                velocityTarget = 0;
                feederIO.setVelocity(0);
            } else {
                velocityTarget = INDEXING_VELOCITY;
                feederIO.setVelocity(INDEXING_VELOCITY);
            }
        });
    }

    public Command indexCommandWithVelocity(double indexingVelocity) {
        return this.run(() -> {
            if (Robot.isSimulation() && RoutingSim.getInstance().getNotePos().isEmpty()) {
                System.out.println("No note loaded");
                return;
            }

            if (beamBreakIOInputs.secondBeamBreak) {
                // Move ring backward
                feederIO.setVelocity(-indexingVelocity / 4);
            } else if (beamBreakIOInputs.firstBeamBreak) {
                feederIO.setVelocity(0);
            } else {
                feederIO.setVelocity(indexingVelocity);
            }
        });
    }

    public Command indexBackwardCmd() {
        return setVelocityCommand(-INDEXING_VELOCITY)
                .until(() -> !(beamBreakIOInputs.firstBeamBreak || beamBreakIOInputs.secondBeamBreak))
                .andThen(indexCommand());
    }

    public BeamBreakIO getBeamBreakIO() {
        return this.beamBreakIO;
    }

    public FeederIOInputs getFeederIOInputs() {
        return this.feederIOInputs;
    }

    public double getFeederMotorCurrentAmps() {
        return feederIOInputs.feederAmps;
    }

}
