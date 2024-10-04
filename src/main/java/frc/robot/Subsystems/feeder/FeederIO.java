// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FeederIO {

    @AutoLog
    public static class FeederIOInputs {
        public double feederVelocityRotationsPerSec = 0.0;
        public double feederVoltage = 0.0;
        public double feederAmps = 0.0;
        public double feederTempC = 0.0;
    }

    public abstract void updateInputs(FeederIOInputs inputs);

    public abstract void setVelocity(double velocityTarget);

    public abstract void setVoltage(double voltage);
}
