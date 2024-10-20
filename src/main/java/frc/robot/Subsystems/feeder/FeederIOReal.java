// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;

/** Add your docs here. */
public class FeederIOReal implements FeederIO {
    private final TalonFX motor = new TalonFX(Constants.FEEDER_ID);

    private final StatusSignal<Double> velocity = motor.getVelocity();
    private final StatusSignal<Double> voltage = motor.getMotorVoltage();
    private final StatusSignal<Double> amps = motor.getStatorCurrent();
    private final StatusSignal<Double> tempC = motor.getDeviceTemp();

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);


    public FeederIOReal() {
        var config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;

        // Copied from repo
        config.Slot0.kV = 0.12;
        config.Slot0.kP = 0.1;

        motor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(50, velocity, voltage, amps, tempC);
        motor.optimizeBusUtilization();
    }

    @Override
    public void setVelocity(double velocityTarget) {
        motor.setControl(velocityVoltage.withVelocity(velocityTarget));
        
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            velocity,
            voltage,
            amps,
            tempC
        );

        inputs.feederAmps = amps.getValue();
        inputs.feederTempC = tempC.getValue();
        inputs.feederVelocityRotationsPerSec = velocity.getValue();
        inputs.feederVoltage = voltage.getValue();
        
    }
    
}
