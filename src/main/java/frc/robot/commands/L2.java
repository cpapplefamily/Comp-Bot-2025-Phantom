// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * L2 command.
 */
public class L2 extends Command {
  
    private WindmillSubsystem m_windmill;
    private ElevatorSubsystem m_elevator;

    /**
     * CoralStation command constructor.
     */
    public L2(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        m_windmill = windmill;
        m_elevator = elevator;
        addRequirements(m_windmill, m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.updateSetpoint(ElevatorCalibrations.kL2Position, false);
        // TODO: Add direcionality here
        if (m_elevator.isWithinTolerance(ElevatorCalibrations.kL2Tolerance)) {
            m_windmill.updateSetpoint(WindmillCalibrations.kL2Position, false);
        }
    }

    @Override
    public void execute() {
        if (m_elevator.isWithinTolerance(ElevatorCalibrations.kL2Tolerance)) {
            m_windmill.updateSetpoint(WindmillCalibrations.kL2Position, false);
        }
    }

    @Override
    public boolean isFinished() {
        return m_elevator.getSetpoint() == ElevatorCalibrations.kL2Position 
            && m_windmill.getSetpoint() == WindmillCalibrations.kL2Position;
    }

}
