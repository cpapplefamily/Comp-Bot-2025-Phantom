// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.Utils;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * L4 command.
 */
public class L4 extends Command {
  
    private WindmillSubsystem m_windmill;
    private ElevatorSubsystem m_elevator;

    /**
     * CoralStation command constructor.
     */
    public L4(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        m_windmill = windmill;
        m_elevator = elevator;
        addRequirements(m_windmill, m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.updateSetpoint(ElevatorCalibrations.kL4Position, false);
        if (m_elevator.isWithinTolerance(ElevatorCalibrations.kL4Tolerance)) {
            m_windmill.updateSetpoint(WindmillCalibrations.kL4Position, false);
        }
    }

    @Override
    public void execute() {
        if (m_elevator.isWithinTolerance(ElevatorCalibrations.kL4Tolerance)) {
            m_windmill.updateSetpoint(WindmillCalibrations.kL4Position, false);
        }
    }

    @Override
    public boolean isFinished() {
        return Utils.isDoubleEqual(m_windmill.getSetpoint(), WindmillCalibrations.kL4Position, 0.01)
               && Utils.isDoubleEqual(m_elevator.getSetpoint(), ElevatorCalibrations.kL4Position, 0.01);
    }

}
