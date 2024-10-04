// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class FlywheelIOSim implements FlywheelIO {

    DCMotorSim flywheelSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), ShooterSubsystem.FLYWHEEL_RATIO, 0.001);

    // Values from GitHub
    private final PIDController flywheelController = new PIDController(0.3, 0.0, 0.0);
    private final SimpleMotorFeedforward motorFF = new SimpleMotorFeedforward(0.0, 0.0925);


    @Override
    public void setVelocityRotationsPerSecond(double velocityTargetRotationsPerSecond) {
        flywheelSim.setInputVoltage(flywheelController.calculate(
            flywheelSim.getAngularVelocityRPM() / 60, velocityTargetRotationsPerSecond)
            + motorFF.calculate(velocityTargetRotationsPerSecond));
    }

    @Override
    public void setVoltage(double voltage) {
        flywheelSim.setInputVoltage(voltage);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        flywheelSim.update(0.02);

        inputs.motorVelocityRotationsPerSecond = flywheelSim.getAngularVelocityRPM() / 60;
        inputs.motorAmps = flywheelSim.getCurrentDrawAmps();
        inputs.motorTempC = 0;
        inputs.motorVoltage = 0;
    }
    
}