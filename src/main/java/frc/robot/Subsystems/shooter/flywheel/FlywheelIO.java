// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FlywheelIO {
    
    @AutoLog
    public static class FlywheelIOInputs {
        public double motorVelocityRotationsPerSecond = 0.0;
        public double motorVoltage = 0.0;
        public double motorAmps = 0.0;
        public double motorTempC = 0.0;
    }

    public abstract void updateInputs(FlywheelIOInputs inputs);

    public abstract void setVelocity(double velocityTargetRotationsPerSecond);

    public abstract void setVoltage(double voltage);
}
