// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder.beambreak;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface BeamBreakIO {

    @AutoLog
    public static class BeamBreakIOInputs {
        public boolean firstBeamBreak = false;
        public boolean secondBeamBreak = false;
    }

    public abstract void updateInputs(BeamBreakIOInputs inputs);
    public abstract boolean getSecondBeamBreak(); 

}
