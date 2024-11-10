// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Subsystems.shooter.ShooterSubsystem;

/** Add your docs here. */
public class PivotIOReal implements PivotIO {

    private final TalonFX pivotMotor = new TalonFX(Constants.PIVOT_TALON_ID, "canivore");

    private final StatusSignal<Double> voltage = pivotMotor.getMotorVoltage();
    private final StatusSignal<Double> currentAmps = pivotMotor.getStatorCurrent();
    private final StatusSignal<Double> tempC = pivotMotor.getDeviceTemp();
    private final StatusSignal<Double> rotations = pivotMotor.getPosition();

    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    private final MotionMagicVoltage pivotMotionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);

    public PivotIOReal() {
        var pivotConfig = new TalonFXConfiguration();

        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        pivotConfig.Feedback.SensorToMechanismRatio = ShooterSubsystem.PIVOT_RATIO;

        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // TODO: Verify & Enable Pivot
        pivotConfig.CurrentLimits.StatorCurrentLimit = 15;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 0;

        // Copied from repo
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Slot0.kG = 0.5;
        pivotConfig.Slot0.kV = 7.2;
        pivotConfig.Slot0.kA = 0.1;
        pivotConfig.Slot0.kS = 0.0;
        pivotConfig.Slot0.kP = 400.0;
        pivotConfig.Slot0.kD = 0.0;

        pivotConfig.MotionMagic.MotionMagicAcceleration = 1.0;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    
        pivotMotor.getConfigurator().apply(pivotConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, voltage, currentAmps, tempC, rotations);
        pivotMotor.optimizeBusUtilization();
    }


    @Override
    public void setPivotSetpoint(Rotation2d setpoint) {
        pivotMotor.setControl(pivotMotionMagic.withPosition(setpoint.getRotations()));
        
    }

    @Override
    public void setVoltage(double targetVoltage) {
        //pivotMotor.setControl(voltageOut.withOutput(targetVoltage));
        pivotMotor.setControl(voltageOut.withOutput(0));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            voltage,
            currentAmps,
            tempC,
            rotations
        );

        inputs.pivotMotorAmps = currentAmps.getValue();
        inputs.pivotMotorVoltage = voltage.getValue();
        inputs.pivotMotorTempC = tempC.getValue();
        inputs.pivotAngle = Rotation2d.fromRotations(rotations.getValue());
        
    }
    
}
