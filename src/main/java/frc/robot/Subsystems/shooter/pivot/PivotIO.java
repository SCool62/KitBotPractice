// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {
        public Rotation2d pivotAngle = new Rotation2d();
        public double pivotMotorVoltage = 0.0;
        public double pivotMotorAmps = 0.0;
        public double pivotMotorTempC = 0.0;
    }

    public abstract void updateInputs(PivotIOInputs inputs);

    public abstract void setPivotSetpoint(Rotation2d setpoint);

    public abstract void setVoltage(double voltage);

}
