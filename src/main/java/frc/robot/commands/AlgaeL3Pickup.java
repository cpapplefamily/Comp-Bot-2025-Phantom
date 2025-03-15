// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * AlgaeFloorPickup command.
 */
public class AlgaeL3Pickup extends SequentialCommandGroup {

    /**
     * AlgaeFloorPickup command constructor.
     */
    public AlgaeL3Pickup(ElevatorSubsystem elevator, WindmillSubsystem windmill, ManipulatorSubsystem manipulator) {
        super(
            // TODO: Add tolerances
            new MoveElevatorToPosition(
                ElevatorCalibrations.kAlgaeL3Position, ElevatorCalibrations.kDefaultTolerance, false, elevator),
            new MoveWindmillToPosition(
                WindmillCalibrations.kAlgaeL3Position, ElevatorCalibrations.kDefaultTolerance, false, windmill),
            new RunAlgaeIntake(manipulator)
        );
    }
}
