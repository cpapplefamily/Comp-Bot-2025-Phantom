// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * OuttakeThenStow command group.
 */
public class CGOuttakeThenStow extends SequentialCommandGroup {

    /**
     * OuttakeThenStow command group Constructor.
     *
     * @param outtakeSpeed the speed at which the piece is outtook
     * @param outtakeTime the time in which the piece is being outtaken for
     * @param elevator the elevator
     * @param windmill the windmill
     * @param manipulator the manipulator
     */
    public CGOuttakeThenStow(double outtakeSpeed, double outtakeTime, 
                            ElevatorSubsystem elevator, WindmillSubsystem windmill, ManipulatorSubsystem manipulator) {
        super(
            new RunManipulator(manipulator, outtakeSpeed).withTimeout(outtakeTime),
            new LollipopStow(elevator, windmill)
        //new CoralStationStow(elevator, windmill)
        );
    }
}
