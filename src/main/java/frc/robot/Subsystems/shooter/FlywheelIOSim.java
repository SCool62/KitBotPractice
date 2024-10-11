// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class FlywheelIOSim implements FlywheelIO {

    DCMotorSim flywheelSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), ShooterSubsystem.FLYWHEEL_RATIO, 0.001);

    // Values from GitHub
    private final PIDController flywheelController = new PIDController(0.3, 0.0, 0.0);
    private final SimpleMotorFeedforward motorFF = new SimpleMotorFeedforward(0.0, 0.0925);

    private double currVoltage;


    @Override
    public void setVelocityRotationsPerSecond(double velocityTargetRotationsPerSecond) {
        double voltage = flywheelController.calculate(
            flywheelSim.getAngularVelocityRPM() / 60, velocityTargetRotationsPerSecond)
            + motorFF.calculate(velocityTargetRotationsPerSecond);
        currVoltage = voltage;
        this.setVoltage(voltage);
    }

    @Override
    public void setVoltage(double voltage) {
        currVoltage = voltage;
        flywheelSim.setInputVoltage(MathUtil.clamp(voltage, -(RobotContainer.getBatteryVoltage()), RobotContainer.getBatteryVoltage()));
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        flywheelSim.update(0.02);

        inputs.motorVelocityRotationsPerSecond = flywheelSim.getAngularVelocityRPM() / 60;
        inputs.motorAmps = flywheelSim.getCurrentDrawAmps();
        inputs.motorTempC = 0;
        inputs.motorVoltage = currVoltage;
    }
    
}
