
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class PivotIOSim implements PivotIO {

    // Values copied from github
    SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), 
    ShooterSubsystem.PIVOT_RATIO, 
    0.85, 
    Units.feetToMeters(12),
     -1.0, 
     2.0, 
     true, 
     0);

     // Values from github
     ProfiledPIDController pidController = new ProfiledPIDController(1.0, 0.0, 1.0, new Constraints(10.0, 10.0));
    ArmFeedforward pivotFF = new ArmFeedforward(0.0, 0.12, 0.8);

    @Override
    public void setPivotSetpoint(Rotation2d setpoint) {
        setVoltage(
            pidController.calculate(pivotSim.getAngleRads(), setpoint.getRadians()) + 
            pivotFF.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity)
        );
    }

    @Override
    public void setVoltage(double voltage) {
        pivotSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));   
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        pivotSim.update(0.02);

        inputs.pivotAngle = Rotation2d.fromRadians(pivotSim.getAngleRads());
        inputs.pivotMotorAmps = pivotSim.getCurrentDrawAmps();
        inputs.pivotMotorVoltage = 0;
        inputs.pivotMotorTempC = 0;
        
    }
    
}
