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
 * LollipopStor command.
 */
public class LollipopStow extends Command {
  
    private WindmillSubsystem m_windmill;
    private ElevatorSubsystem m_elevator;

    /**
     * LollipopStor command constructor.
     *
     * <p>1. Move the arm to 90deg +- frame perimeter tolerance...this assumes the arm will ALWAYS go travale the
     *    shortest path (doesn't swing through 180deg)
     *
     * <p>2. Move the elevator to fully down
     */
    public LollipopStow(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        m_windmill = windmill;
        m_elevator = elevator;
        addRequirements(m_windmill, m_elevator);
    }

    @Override
    public void initialize() {
        m_windmill.updateSetpoint(WindmillCalibrations.kLollipopPosition, false);
        if (m_windmill.isWithinTolerance(WindmillCalibrations.kLollipopTolerance)) {
            m_elevator.updateSetpoint(ElevatorCalibrations.kBottomPosition, false);
        }
    }
  
    @Override
     public void execute() {
        if (m_windmill.isWithinTolerance(WindmillCalibrations.kLollipopTolerance)) {
            m_elevator.updateSetpoint(ElevatorCalibrations.kBottomPosition, false);
        }
    }
  
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return Utils.isDoubleEqual(m_windmill.getSetpoint(), WindmillCalibrations.kLollipopPosition, 0.01)
               && Utils.isDoubleEqual(m_elevator.getSetpoint(), ElevatorCalibrations.kBottomPosition, 0.01);
    }

}
