// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder.feeder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.feeder.RoutingSim;

/** Add your docs here. */
public class FeederIOSim implements FeederIO {

    TalonFX feederTalonFX = new TalonFX(0);

    DCMotorSim feederMotorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 12f / 22f, 0.01);

    TalonFXSimState feederTalonFXSim = feederTalonFX.getSimState();

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);


    public FeederIOSim() {
        TalonFXConfiguration toConfigure = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

        currentLimits.SupplyCurrentLimit = 20
        ;
        currentLimits.SupplyCurrentLimitEnable = true; // And enable it

        toConfigure.CurrentLimits = currentLimits;
        // Original: kp = 0.1, kV = 0.055
        toConfigure.Slot0.kP = 0.1;
        toConfigure.Slot0.kV = 0.068;

        feederTalonFX.getConfigurator().apply(toConfigure);

    }


    @Override
    public void setVelocity(double velocityTarget) {
        feederTalonFX.setControl(velocityVoltage.withVelocity(velocityTarget));
    }

    @Override
    public void setVoltage(double voltage) {
        feederTalonFX.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        feederTalonFXSim.setSupplyVoltage(RobotContainer.getBatteryVoltage());

        feederMotorSim.setInput(feederTalonFXSim.getMotorVoltage());

        feederMotorSim.update(0.02);

        if (RoutingSim.getInstance().getNotePos().isEmpty() || RoutingSim.getInstance().getNotePos().get() < (0.051 + Units.inchesToMeters(14))
        ) {
            RoutingSim.getInstance().updateSim(0.02, feederMotorSim.getAngularVelocityRPM() / 60, 0.0508);
        }
        
        inputs.feederAmps = feederTalonFXSim.getSupplyCurrent();
        inputs.feederVelocityRotationsPerSec = feederMotorSim.getAngularVelocityRPM() / 60;
        inputs.feederVoltage = feederTalonFXSim.getMotorVoltage();
        inputs.feederTempC = 0;
    }
    
}
