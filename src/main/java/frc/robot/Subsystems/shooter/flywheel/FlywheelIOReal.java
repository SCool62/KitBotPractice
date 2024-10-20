// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Subsystems.shooter.ShooterSubsystem;


/** Add your docs here. */
public class FlywheelIOReal implements FlywheelIO {

    //private final int motorID;
    private final TalonFX flywheelMotor;

    private final StatusSignal<Double> velocity;
    private final StatusSignal<Double> currentAmps;
    private final StatusSignal<Double> voltage;
    private final StatusSignal<Double> tempC;


    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    // PID Loop on Velocity
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);


    public FlywheelIOReal(int motorID, InvertedValue motorInvertConfig) {
        //this.motorID = motorID;

        flywheelMotor = new TalonFX(motorID);

        // Initialize inputs
        velocity = flywheelMotor.getVelocity();
        currentAmps = flywheelMotor.getStatorCurrent();
        voltage = flywheelMotor.getMotorVoltage();
        tempC = flywheelMotor.getDeviceTemp();

        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.Inverted = motorInvertConfig;

        flywheelConfig.Feedback.SensorToMechanismRatio = ShooterSubsystem.FLYWHEEL_RATIO;

        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

        // Copied from repo
        flywheelConfig.Slot0.kA = 0.0051316;
        flywheelConfig.Slot0.kV = 0.095;
        flywheelConfig.Slot0.kS = 0.3;
        flywheelConfig.Slot0.kP = 0.1;
        flywheelConfig.Slot0.kD = 0.0;

        flywheelMotor.getConfigurator().apply(flywheelConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50, velocity, voltage, currentAmps, tempC);
        flywheelMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            velocity,
            currentAmps,
            voltage,
            tempC
        );

        inputs.motorVelocityRotationsPerSecond = velocity.getValue();
        inputs.motorAmps = currentAmps.getValue();
        inputs.motorVoltage = voltage.getValue();
        inputs.motorTempC = tempC.getValue();
        
    }

    @Override
    public void setVelocity(double velocityTargetRotationsPerSecond) {
        flywheelMotor.setControl(velocityVoltage.withVelocity(velocityTargetRotationsPerSecond));
    }

    @Override
    public void setVoltage(double voltage) {
        flywheelMotor.setControl(voltageOut.withOutput(voltage));
    }
    
}
