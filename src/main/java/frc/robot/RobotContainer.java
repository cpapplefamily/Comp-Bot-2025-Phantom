// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Calibrations.ElevatorLockCalibrations;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.commands.CGOuttakeThenStow;
import frc.robot.commands.CoralStation;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.LollipopStow;
import frc.robot.commands.PendulumStow;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TranslationAlignToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorLockSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * The robot container.
 */
public class RobotContainer {

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    public final ElevatorLockSubsystem m_elevatorLock = new ElevatorLockSubsystem();
    public final WindmillSubsystem m_windmill = new WindmillSubsystem();
    public final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();
    private final CommandXboxController m_joystick = new CommandXboxController(0);

    /** TODO: Look into using DriveRequestType.Velocity */
    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
            .withDeadband(Calibrations.DriverCalibrations.kmaxSpeed * 0.1)
            .withRotationalDeadband(Calibrations.DriverCalibrations.kmaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * The robot container constructor.
     */
    public RobotContainer() {
        configureBindings();
    }

    /**
     * Configure bindings defines how the robot is controlled via user input.
     */
    private void configureBindings() {

        m_drivetrain.setDefaultCommand(
            m_drivetrain.applyRequest(() -> m_drive
                .withVelocityX(-m_joystick.getLeftY() * Calibrations.DriverCalibrations.kmaxSpeed)
                .withVelocityY(-m_joystick.getLeftX() * Calibrations.DriverCalibrations.kmaxSpeed)
                .withRotationalRate(-m_joystick.getRightX() * Calibrations.DriverCalibrations.kmaxAngularRate)
            )
        );
    
        /* This allows the driver to reset the rotation */
        m_joystick.start().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

        m_joystick.b().onTrue(new LollipopStow(m_elevator, m_windmill));

        m_joystick.x().onTrue(new PendulumStow(m_elevator, m_windmill));
    
        /* Coral station pickup sequence */
        m_joystick.rightBumper().onTrue(new CoralStation(m_elevator, m_windmill));
        m_joystick.rightBumper().whileTrue(new RunIntake(m_manipulator));
        m_joystick.rightBumper().onFalse(new LollipopStow(m_elevator, m_windmill));

        /* Coral reef L4 dropoff sequence */
        m_joystick.povUp().onTrue(new L4(m_elevator, m_windmill));
        m_joystick.povUp().onFalse(new CGOuttakeThenStow(ManipulatorCalibrations.kL4OuttakeSpeed, 
                                                         ManipulatorCalibrations.kL4OuttakeTime, 
                                                         m_elevator, m_windmill, m_manipulator));

        /* Coral reef L3 dropoff sequence */
        m_joystick.povLeft().onTrue(new L3(m_elevator, m_windmill));
        m_joystick.povLeft().onFalse(new CGOuttakeThenStow(ManipulatorCalibrations.kL3OuttakeSpeed,
                                                           ManipulatorCalibrations.kL3OuttakeTime,
                                                           m_elevator, m_windmill, m_manipulator));

        /* Coral reef L2 dropoff sequence */
        m_joystick.povRight().onTrue(new L2(m_elevator, m_windmill));
        m_joystick.povRight().onFalse(new CGOuttakeThenStow(ManipulatorCalibrations.kL2OuttakeSpeed,
                                                           ManipulatorCalibrations.kL2OuttakeTime,
                                                           m_elevator, m_windmill, m_manipulator));
        // /* Target the left coral reef stick */
        // m_joystick.axisGreaterThan(2, 0.1).whileTrue(new RotationAlignToTag(1,
        //                                                                            () -> m_joystick.getLeftY(), 
        //                                                                            () -> m_joystick.getLeftX(), 
        //                                                                            m_drivetrain));
        // /* Target the right coral reef stick */
        // m_joystick.axisGreaterThan(3, 0.1).whileTrue(new RotationAlignToTag(0,
        //                                                                            () -> m_joystick.getLeftY(), 
        //                                                                            () -> m_joystick.getLeftX(), 
        //                                                                            m_drivetrain));

        /* Target the left coral reef stick */
        m_joystick.axisGreaterThan(2, 0.1).whileTrue(new TranslationAlignToTag(1, 
                                                                                   m_drivetrain));
        /* Target the right coral reef stick */
        m_joystick.axisGreaterThan(3, 0.1).whileTrue(new TranslationAlignToTag(0,
                                                                                   m_drivetrain));

        m_joystick.y().onTrue(new InstantCommand(() -> m_elevatorLock.setAngle(ElevatorLockCalibrations.kservoLockAngle)));


    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
