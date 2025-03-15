// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.Utils;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * Pendulum Stow Command.
 */
public class PendulumStow extends Command {

    private WindmillSubsystem m_windmill;
    private ElevatorSubsystem m_elevator;

    /**
     * Pendulum Stow Command Constructor.
     */
    public PendulumStow(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        // TODO: Make this a sequential command group
        m_windmill = windmill;
        m_elevator = elevator;
        addRequirements(m_windmill, m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.updateSetpoint(ElevatorCalibrations.kPendulumPosition, false);
        if (m_elevator.isWithinTolerance(ElevatorCalibrations.kPendulumTolerance)) {
            m_windmill.updateSetpoint(WindmillCalibrations.kPendulumPosition, false);
        }

        new InstantCommand(() -> m_elevator.setAngle(ElevatorCalibrations.kservoUnlockAngle));
    }

    @Override
    public void execute() {
        if (m_elevator.isWithinTolerance(ElevatorCalibrations.kPendulumTolerance)) {
            m_windmill.updateSetpoint(WindmillCalibrations.kPendulumPosition, false);
        }
    }

    @Override
    public boolean isFinished() {
        return Utils.isDoubleEqual(m_windmill.getSetpoint(), WindmillCalibrations.kPendulumPosition, 0.01)
               && Utils.isDoubleEqual(m_elevator.getSetpoint(), ElevatorCalibrations.kPendulumPosition, 0.01);
    }
}
