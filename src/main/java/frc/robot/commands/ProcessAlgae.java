// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * CoralStation command.
 */
public class ProcessAlgae extends SequentialCommandGroup {

    /**
     * CoralStation command constructor.
     */
    public ProcessAlgae(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        super(

            new MoveWindmillToPosition(
                WindmillCalibrations.kProcessorPosition, 
                WindmillCalibrations.kProcessorTolerance, 
                false, windmill),
            new MoveElevatorToPosition(
                ElevatorCalibrations.kProcessorPosition, 
                ElevatorCalibrations.kProcessorTolerance, 
                false, elevator)

        );
    }

}
