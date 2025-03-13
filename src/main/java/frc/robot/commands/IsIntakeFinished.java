// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * IsIntakeFinished command.
 */
public class IsIntakeFinished extends Command {
    ManipulatorSubsystem m_manipulator;

    /**
     * IsIntakeFinished command constructor.
     *
     * @param manipulator the manipulator
     */
    public IsIntakeFinished(ManipulatorSubsystem manipulator) {
        m_manipulator = manipulator;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_manipulator.hasCoral();
    }
}
