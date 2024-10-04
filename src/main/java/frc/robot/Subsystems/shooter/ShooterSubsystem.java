// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import java.util.function.DoubleSupplier;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.feeder.RoutingSim;

/** Add your docs here. */
public class ShooterSubsystem extends SubsystemBase {
    // Copied from repo
    public static final double FLYWHEEL_RATIO = 18.0 / 24.0;
    public static final double PIVOT_RATIO = (27.0 / 1.0) * (48.0 / 22.0);

    private static final double NOTE_POS_FLYWHEEL_LOWER_LIMIT = 0.204;

    // IO Interfaces
    private final FlywheelIO flywheelIOLeft;
    private final FlywheelIO flywheelIORight;
    private final PivotIO pivotIO;

    // Inputs
    private final FlywheelIOInputsAutoLogged flywheelIOInputsLeft = new FlywheelIOInputsAutoLogged();
    private final FlywheelIOInputsAutoLogged flywheelIOInputsRight = new FlywheelIOInputsAutoLogged();
    private final PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();


    private double leftGoal = 0.0;
    private double rightGoal = 0.0;
    private Rotation2d pivotGoal = new Rotation2d();
    

    public ShooterSubsystem(FlywheelIO flywheelIOLeft, FlywheelIO flywheelIORight, PivotIO pivotIO) {
        this.flywheelIOLeft = flywheelIOLeft;
        this.flywheelIORight = flywheelIORight;
        this.pivotIO = pivotIO;

    }

    
    @Override
    public void periodic() {
        super.periodic();
        // Update inputs
        flywheelIOLeft.updateInputs(flywheelIOInputsLeft);
        flywheelIORight.updateInputs(flywheelIOInputsRight);
        pivotIO.updateInputs(pivotIOInputs);

        if ((!RoutingSim.getInstance().getNotePos().isEmpty() && RoutingSim.getInstance().getNotePos().get() >= NOTE_POS_FLYWHEEL_LOWER_LIMIT)) {
            RoutingSim.getInstance().updateSim(0.02, flywheelIOInputsLeft.motorVelocityRotationsPerSecond);
        }

        Logger.processInputs("Flywheel1", flywheelIOInputsLeft);
        Logger.processInputs("Flywheel2", flywheelIOInputsRight);
        Logger.processInputs("Pivot", pivotIOInputs);
        Logger.recordOutput("PivotGoal", pivotGoal);
        Logger.recordOutput("Flywheel1Goal", leftGoal);
        Logger.recordOutput("Flywheel2Goal", rightGoal);

    }

    public Command runShooterCommand(Rotation2d pivotSetpoint, DoubleSupplier left, DoubleSupplier right) {
        return this.run(() -> {
            leftGoal = left.getAsDouble();
            rightGoal = right.getAsDouble();
            pivotGoal = pivotSetpoint;

            Logger.recordOutput("Shooter/Left Velocity Setpoint", left.getAsDouble());
            Logger.recordOutput("Shooter/Right Velocity Setpoint", right.getAsDouble());
            Logger.recordOutput("Shooter/Pivot Goal", pivotSetpoint);
            Logger.recordOutput(
              "Shooter/Left At Target",
              MathUtil.isNear(
                  left.getAsDouble(), flywheelIOInputsLeft.motorVelocityRotationsPerSecond, 1.0));
            Logger.recordOutput(
              "Shooter/Right At Target",
              MathUtil.isNear(
                  right.getAsDouble(), flywheelIOInputsRight.motorVelocityRotationsPerSecond, 1.0));
            

            flywheelIOLeft.setVelocityRotationsPerSecond(leftGoal);
            flywheelIORight.setVelocityRotationsPerSecond(rightGoal);
            pivotIO.setPivotSetpoint(pivotGoal);
        });
    }

    public Command setPivotAngleCommand(Rotation2d setpoint) {
        return this.run(() -> {
            pivotGoal = setpoint;
            Logger.recordOutput("Shooter/Pivot Goal", pivotGoal);
            pivotIO.setPivotSetpoint(pivotGoal);
        });
    }

    public Command setFlywheelVelocityCommand(DoubleSupplier left, DoubleSupplier right) {
        return this.run(() -> {
            leftGoal = left.getAsDouble();
            rightGoal = right.getAsDouble();
            flywheelIOLeft.setVelocityRotationsPerSecond(left.getAsDouble());
            flywheelIORight.setVelocityRotationsPerSecond(right.getAsDouble());
        });
    }

}
