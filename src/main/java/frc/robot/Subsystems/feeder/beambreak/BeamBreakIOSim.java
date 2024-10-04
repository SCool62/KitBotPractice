// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder.beambreak;

/** Add your docs here. */
public class BeamBreakIOSim implements BeamBreakIO {

    private boolean firstBeamBreak = false;
    private boolean secondBeamBreak = false;

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.firstBeamBreak = firstBeamBreak;
        inputs.secondBeamBreak = secondBeamBreak;
    }

    public void setFirstBeamBreakState(boolean state) {
        firstBeamBreak = state;
    }

    public void setSecondBeamBreakState(boolean state) {
        secondBeamBreak = state;
    }

    @Override
    public boolean getSecondBeamBreak() {
        // TODO Auto-generated method stub
        return secondBeamBreak;
    }

    
    
}
