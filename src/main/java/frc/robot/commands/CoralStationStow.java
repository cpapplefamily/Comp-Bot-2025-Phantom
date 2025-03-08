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
 * CoralStationStow command.
 */
public class CoralStationStow extends Command {
  
    private WindmillSubsystem m_windmill;
    private ElevatorSubsystem m_elevator;

    /**
     * CoralStationStow command constructor.
     */
    public CoralStationStow(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        m_windmill = windmill;
        m_elevator = elevator;
        addRequirements(m_windmill, m_elevator);
    }

    @Override
    public void initialize() {
        m_windmill.updateSetpoint(WindmillCalibrations.kCoralStationStowPosition, false);
        if (m_windmill.isWithinTolerance(WindmillCalibrations.kCoralStationStowTolerance)) {
            m_elevator.updateSetpoint(ElevatorCalibrations.kCoralStationStowPosition, false);
        }
    }

    @Override
    public void execute() {
        if (m_windmill.isWithinTolerance(WindmillCalibrations.kCoralStationStowTolerance)) {
            m_elevator.updateSetpoint(ElevatorCalibrations.kCoralStationStowPosition, false);
        }
    }

    @Override
    public boolean isFinished() {
        return m_elevator.getSetpoint() == ElevatorCalibrations.kCoralStationStowPosition 
            && m_windmill.getSetpoint() == WindmillCalibrations.kCoralStationStowPosition;
    }

}
