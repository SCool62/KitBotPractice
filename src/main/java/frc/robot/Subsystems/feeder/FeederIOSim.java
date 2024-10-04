// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class FeederIOSim implements FeederIO {

    DCMotorSim feederMotorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 12f / 22f, 0.01);

    private final PIDController feederPid = new PIDController(10, 0.0, 0.0);
    // kV = krakenKV (0.12) * gear ratio
    private final SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(0.0, (0.12) * 12 / 22);

    public FeederIOSim() {}



    @Override
    public void setVelocity(double velocityTarget) {
        feederMotorSim.setInputVoltage(feederPid.calculate(
            feederMotorSim.getAngularVelocityRPM() / 60, velocityTarget)
            + feederFF.calculate(velocityTarget));
    }

    @Override
    public void setVoltage(double voltage) {
        feederMotorSim.setInputVoltage(voltage);  
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        feederMotorSim.update(0.02);
        RoutingSim.getInstance().updateSim(0.02, feederMotorSim.getAngularVelocityRPM() / 60);
        
        inputs.feederAmps = feederMotorSim.getCurrentDrawAmps();
        inputs.feederVelocityRotationsPerSec = feederMotorSim.getAngularVelocityRPM() / 60;
        inputs.feederVoltage = 0;
        inputs.feederTempC = 0;
    }
    
}
