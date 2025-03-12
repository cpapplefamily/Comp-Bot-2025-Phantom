// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.Utils;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * CoralStation command.
 */
public class CoralStation extends SequentialCommandGroup {

    /**
     * CoralStation command constructor.
     */
    public CoralStation(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        super(
            new ParallelCommandGroup(
                new MoveWindmillToPosition(
                    WindmillCalibrations.kCoralStationPrepPosition, 
                    WindmillCalibrations.kCoralStationPrepTolerance, 
                    false, windmill),
                new MoveElevatorToPosition(
                    ElevatorCalibrations.kCoralStationPosition, 
                    ElevatorCalibrations.kCoralStationTolerance, 
                    false, elevator)
            ),
            new MoveWindmillToPosition(WindmillCalibrations.kCoralStationPosition, 10, false, windmill)
        );
    }

}
