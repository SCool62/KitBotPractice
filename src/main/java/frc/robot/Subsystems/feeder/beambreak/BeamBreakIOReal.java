// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder.beambreak;

import frc.robot.Constants;
import frc.robot.util.InvertedDigitalInput;

/** Add your docs here. */
public class BeamBreakIOReal implements BeamBreakIO {

    final InvertedDigitalInput firstBeamBreak = new InvertedDigitalInput(Constants.FIRST_BEAMBREAK_CH);
    final InvertedDigitalInput secondBeamBreak = new InvertedDigitalInput(Constants.SECOND_BEAMBREAK_CH);

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.firstBeamBreak = firstBeamBreak.get();
        inputs.secondBeamBreak = secondBeamBreak.get();
    }

    @Override
    public boolean getSecondBeamBreak() {
        // TODO Auto-generated method stub
        return secondBeamBreak.get();
    }

    @Override
    public boolean getFirstBeamBreak() {
        return firstBeamBreak.get();
    }

    

    

}
