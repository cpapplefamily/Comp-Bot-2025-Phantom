// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralStation;
import frc.robot.commands.LollipopStow;
import frc.robot.commands.PendulumStow;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * The robot container.
 */
public class RobotContainer {

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    public final WindmillSubsystem m_windmill = new WindmillSubsystem();
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
    
        m_joystick.rightBumper().onTrue(new CoralStation(m_elevator, m_windmill));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
