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
import frc.robot.Subsystems.feeder.RoutingSim;
import frc.robot.Subsystems.shooter.ShooterSubsystem;

/** Add your docs here. */
public class FlywheelIOSim implements FlywheelIO {

    public static final double NOTE_POS_FLYWHEEL_LOWER_LIMIT = 0.204;

    TalonFX flyweelTalonFX;
    TalonFXSimState talonFxSimState;

    DCMotorSim flywheelSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), ShooterSubsystem.FLYWHEEL_RATIO, 0.01);

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

    private boolean updatesRoutingSim = false;

    public FlywheelIOSim(int deviceId) {
        flyweelTalonFX = new TalonFX(deviceId);
        talonFxSimState = flyweelTalonFX.getSimState();

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

        currentLimits.SupplyCurrentLimit = 20;
        currentLimits.SupplyCurrentLimitEnable = true;

        configuration.CurrentLimits = currentLimits;

        configuration.Slot0.kP = 0.0;
        configuration.Slot0.kV = 0.094;

        flyweelTalonFX.getConfigurator().apply(configuration);
    }


    @Override
    public void setVelocity(double velocityTargetRotationsPerSecond) {
        flyweelTalonFX.setControl(velocityVoltage.withVelocity(velocityTargetRotationsPerSecond));
    }

    @Override
    public void setVoltage(double voltage) {
        flyweelTalonFX.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        talonFxSimState.setSupplyVoltage(RobotContainer.getBatteryVoltage());

        flywheelSim.setInput(talonFxSimState.getMotorVoltage());

        flywheelSim.update(0.02);

        if ((!RoutingSim.getInstance().getNotePos().isEmpty() && RoutingSim.getInstance().getNotePos().get() >= NOTE_POS_FLYWHEEL_LOWER_LIMIT)) {
            RoutingSim.getInstance().updateSim(0.02, flywheelSim.getAngularVelocityRPM() / 60, 0.100713);
        }

        inputs.motorVelocityRotationsPerSecond = flywheelSim.getAngularVelocityRPM() / 60;
        inputs.motorAmps = talonFxSimState.getSupplyCurrent();
        inputs.motorTempC = 0;
        inputs.motorVoltage = talonFxSimState.getMotorVoltage();
    }

    public boolean updatesRoutingSim() {
        return updatesRoutingSim;
    }

    public void setUpdatesRoutingSim(boolean updatesRoutingSim) {
        this.updatesRoutingSim = updatesRoutingSim;
    }
}
