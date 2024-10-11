// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.feeder.beambreak.BeamBreakIOSim;

/** Add your docs here. */
public class RoutingSim {

    private static double LOWER_LIMIT_BEAMBREAK_ONE = Units.inchesToMeters(8);
    private static double UPPER_LIMIT_BEAMBREAK_ONE = Units.inchesToMeters(22.65);
    private static double LOWER_LIMIT_BEAMBREAK_TWO = Units.inchesToMeters(8.85);
    private static double UPPER_LIMIT_BEAMBREAK_TWO = Units.inchesToMeters(23.5);

    private Optional<BeamBreakIOSim> beamBreakIO = Optional.empty();
    

    private Optional<Double> notePos = Optional.empty();

    private final static RoutingSim instance = new RoutingSim();

    private RoutingSim() {}


    public void updateSim(double dtSeconds, double velocityRotationPerSec) {
        
        if (notePos.isEmpty()) {
            Logger.recordOutput("SimNotePos", -1.0);
            beamBreakIO.get().setFirstBeamBreakState(false);
            beamBreakIO.get().setSecondBeamBreakState(false);
            return;
        }

        if (notePos.get() > (0.450 + Units.inchesToMeters(14))) {
            notePos = Optional.empty();
            return;
        }
        
        // 1.562 is the diameter of the roller
        double velocityMetersPerSec = Units.inchesToMeters(velocityRotationPerSec * (Units.inchesToMeters(1.562) * Math.PI));

        notePos = Optional.of(notePos.get() + (velocityMetersPerSec * dtSeconds));

        Logger.recordOutput("SimNotePos", notePos.get());

        if (notePos.get() < LOWER_LIMIT_BEAMBREAK_ONE || notePos.get() > UPPER_LIMIT_BEAMBREAK_ONE) {
            beamBreakIO.get().setFirstBeamBreakState(false);
        } else {
            beamBreakIO.get().setFirstBeamBreakState(true);
        }


        if (notePos.get() < LOWER_LIMIT_BEAMBREAK_TWO || notePos.get() > UPPER_LIMIT_BEAMBREAK_TWO) {
            beamBreakIO.get().setSecondBeamBreakState(false);
        } else {
            beamBreakIO.get().setSecondBeamBreakState(true);
        }
        
    }

    public void setNotePos(Optional<Double> pos) {
        notePos = pos;
    }

    public void setBeamBreakIOSim(BeamBreakIOSim beamBreakIOSim) {
        this.beamBreakIO = Optional.of(beamBreakIOSim);
    }

    public static RoutingSim getInstance() {
        return instance;
    }

    public Optional<Double> getNotePos() {
        return notePos;
    }
}
