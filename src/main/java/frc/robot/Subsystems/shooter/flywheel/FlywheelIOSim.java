// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.shooter.ShooterSubsystem;

/** Add your docs here. */
public class FlywheelIOSim implements FlywheelIO {

    TalonFX flyweelTalonFX = new TalonFX(0);
    TalonFXSimState simState = flyweelTalonFX.getSimState();

    DCMotorSim flywheelSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), ShooterSubsystem.FLYWHEEL_RATIO, 0.001);

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

    public FlywheelIOSim() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

        currentLimits.SupplyCurrentLimit = 60;
        currentLimits.SupplyCurrentLimitEnable = true;

        configuration.CurrentLimits = currentLimits;

        // Values from Github
        configuration.Slot0.kP = 0.3;
        configuration.Slot0.kV = 0.0925;

        flyweelTalonFX.getConfigurator().apply(configuration);
    }


    @Override
    public void setVelocity(double velocityTargetRotationsPerSecond) {
        System.out.println("Setting Flywheel Velocity.");
        flyweelTalonFX.setControl(velocityVoltage.withVelocity(velocityTargetRotationsPerSecond));
    }

    @Override
    public void setVoltage(double voltage) {
        System.out.println("Setting Flywheel Voltage");
        flyweelTalonFX.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        simState.setSupplyVoltage(RobotContainer.getBatteryVoltage());

        flywheelSim.setInput(simState.getMotorVoltage());

        flywheelSim.update(0.02);

        inputs.motorVelocityRotationsPerSecond = flywheelSim.getAngularVelocityRPM() / 60;
        inputs.motorAmps = simState.getSupplyCurrent();
        inputs.motorTempC = 0;
        inputs.motorVoltage = simState.getMotorVoltage();
    }
    
}
