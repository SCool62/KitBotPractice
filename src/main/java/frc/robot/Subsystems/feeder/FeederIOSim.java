// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class FeederIOSim implements FeederIO {

    DCMotorSim feederMotorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 12f / 22f, 0.01);

    private final PIDController feederPid = new PIDController(10, 0.0, 0.0);
    // kV = krakenKV (0.12) * gear ratio
    private final SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(0.0, (0.12) * 12 / 22);

    private double currVoltage = 0.0;

    public FeederIOSim() {}



    @Override
    public void setVelocity(double velocityTarget) {
        setVoltage(feederPid.calculate(
            feederMotorSim.getAngularVelocityRPM() / 60, velocityTarget)
            + feederFF.calculate(velocityTarget));
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -(RobotContainer.getBatteryVoltage()), RobotContainer.getBatteryVoltage());
        currVoltage = voltage;
        feederMotorSim.setInputVoltage(voltage);  
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        feederMotorSim.update(0.02);

        if (RoutingSim.getInstance().getNotePos().isEmpty() || RoutingSim.getInstance().getNotePos().get() < (0.051 + Units.inchesToMeters(14))
        ) {
            RoutingSim.getInstance().updateSim(0.02, feederMotorSim.getAngularVelocityRPM() / 60);
        }
        
        inputs.feederAmps = feederMotorSim.getCurrentDrawAmps();
        inputs.feederVelocityRotationsPerSec = feederMotorSim.getAngularVelocityRPM() / 60;
        inputs.feederVoltage = currVoltage;
        inputs.feederTempC = 0;
    }
    
}
