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
 * Deep climb placement command group.
 */
public class PrepClimb extends Command {

    private final ElevatorSubsystem m_elevator;
    private final WindmillSubsystem m_windmill;

    /** Deep climb placement command group constructor.
     *
     * @param elevator the elevator
     * @param windmill the windmill
     */
    public PrepClimb(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        m_elevator = elevator;
        m_windmill = windmill;
        addRequirements(m_windmill, m_elevator);
    }

    @Override
    public void initialize() {
        m_windmill.updateSetpoint(WindmillCalibrations.kClimbPosition, false);
        if (m_windmill.isWithinTolerance(WindmillCalibrations.kClimbTolerance)) {
            m_elevator.updateSetpoint(ElevatorCalibrations.kClimbPosition, false);
        }
    }
  
    @Override
     public void execute() {
        if (m_windmill.isWithinTolerance(WindmillCalibrations.kClimbTolerance)) {
            m_elevator.updateSetpoint(ElevatorCalibrations.kClimbPosition, false);
        }
    }
  
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return m_windmill.getSetpoint() == WindmillCalibrations.kClimbPosition 
            && m_elevator.getSetpoint() == ElevatorCalibrations.kClimbPosition;
    }
}
