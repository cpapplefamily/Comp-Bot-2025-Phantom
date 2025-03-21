// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * RunIntake.
 */
public class RunIntake extends Command {

    private ManipulatorSubsystem m_manipulator;

    private boolean m_isAtVelocity = false;
    
    /**
     * RunIntake command constructor.
     */
    public RunIntake(ManipulatorSubsystem manipulator) {
        m_manipulator = manipulator;
        addRequirements(m_manipulator);
    }

    @Override
    public void initialize() {
        m_manipulator.changeCoralState(false);
        m_isAtVelocity = false;
        m_manipulator.updateSetpoint(ManipulatorCalibrations.kMaxSpeed, ManipulatorCalibrations.kCoralAcceleration);
    }

    @Override
    public void execute() {
        if (Math.abs(m_manipulator.getVelocity() - ManipulatorCalibrations.kMaxSpeed) 
            < ManipulatorCalibrations.kIntakeVelocityTolerance) {

            m_isAtVelocity = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.updateSetpoint(0, ManipulatorCalibrations.kCoralAcceleration);
        m_manipulator.changeCoralState(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (m_isAtVelocity 
            && (Math.abs(m_manipulator.getVelocity()) < ManipulatorCalibrations.kIntakeZeroTolerance));
    } 

}
