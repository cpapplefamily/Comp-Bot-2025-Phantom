// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * Intake command.
 */
public class Intake extends SequentialCommandGroup {
    
    /**
     * CoralStation command constructor.
     */
    public Intake(ElevatorSubsystem elevator, WindmillSubsystem windmill, ManipulatorSubsystem manipulator) {
    
        super(
            new CoralStation(elevator, windmill),
            new InstantCommand(() -> manipulator.updateSetpoint(1.0))
        );

    // super(
    //   new ConditionalCommand(
    //     new SetWindmillSetpoint(90, 5, false, false, elevator, windmill),
    //     new ConditionalCommand(
    //       new InstantCommand(),
    //       new Retract(windmill, elevator),
    //       () -> (windmillSetpoint < 170 && windmillSetpoint > 10)
    //     ),
    //     () -> elevator.getPosition() > elevatorHeight
    //   ), 
    //   new SetElevatorSetpoint(elevatorHeight, Calibrations.PlacementCalibrations.kElevatorTolerance, false, elevator, windmill),
    //   new SetWindmillSetpoint(windmillSetpoint, Calibrations.PlacementCalibrations.kWindmillTolerance, false, false, elevator, windmill)
    // );    }
}
