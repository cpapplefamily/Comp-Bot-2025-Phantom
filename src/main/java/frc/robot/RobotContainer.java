// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Calibrations.DriverCalibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.commands.AlgaeFloorPickup;
import frc.robot.commands.AlgaeL2Pickup;
import frc.robot.commands.AlgaeL3Pickup;
import frc.robot.commands.BargeAlgae;
import frc.robot.commands.CGClimb;
import frc.robot.commands.CGOuttakeThenStow;
import frc.robot.commands.CoralStation;
import frc.robot.commands.IsIntakeFinished;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.LollipopStow;
import frc.robot.commands.PendulumStow;
import frc.robot.commands.PrepClimb;
import frc.robot.commands.ProcessAlgae;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunManipulator;
import frc.robot.commands.TranslationAlignToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.WindmillSubsystem;

/**
 * The robot container.
 */
public class RobotContainer {

    /* Create the robot subsystems */
    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    public final WindmillSubsystem m_windmill = new WindmillSubsystem();
    public final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();
    public final LEDSubsystem m_leds = new LEDSubsystem();
    public final CommandXboxController m_joystick = new CommandXboxController(0);

    /* Drive request for default drivetrain command */
    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
            .withDeadband(Calibrations.DriverCalibrations.kmaxSpeed * 0.1)
            .withRotationalDeadband(Calibrations.DriverCalibrations.kmaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* Path follower */
    private final SendableChooser<Command> m_autoChooser;

    /**
     * The robot container constructor.
     */
    public RobotContainer() {
        SmartDashboard.putData("Unlock", new InstantCommand(
            () -> m_elevator.setAngle(ElevatorCalibrations.kservoUnlockAngle)));

        SmartDashboard.putData("Lock", new InstantCommand(
            () -> m_elevator.setAngle(ElevatorCalibrations.kservoLockAngle))); 

        configureBindings();

        new EventTrigger("L4").onTrue(new L4(m_elevator, m_windmill)
            .andThen(new CGOuttakeThenStow(
                ManipulatorCalibrations.kL4OuttakeSpeed, 
                ManipulatorCalibrations.kL4OuttakeTime, 
                m_elevator, m_windmill, m_manipulator)));
        new EventTrigger("Lollipop Stow").onTrue(new LollipopStow(m_elevator, m_windmill));
        new EventTrigger("Intake").onTrue(new CoralStation(m_elevator, m_windmill).alongWith(new RunIntake(m_manipulator)));

        NamedCommands.registerCommand("Is Intake Finished?", new IsIntakeFinished(m_manipulator));
        
        m_autoChooser = AutoBuilder.buildAutoChooser("3m test path");
        SmartDashboard.putData("Auto Mode", m_autoChooser);
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
        m_joystick.rightBumper().onTrue(new RunIntake(m_manipulator)
            .andThen(new InstantCommand(
                () -> m_joystick.setRumble(RumbleType.kBothRumble, DriverCalibrations.kControllerRumbleValue)))
            .andThen(new WaitCommand(DriverCalibrations.kControllerRumblePulseTime))
            .andThen(new InstantCommand(
                () -> m_joystick.setRumble(RumbleType.kBothRumble, 0))));
        m_joystick.rightBumper().onFalse(new LollipopStow(m_elevator, m_windmill));

        /* Coral reef L4 dropoff sequence */
        m_joystick.povUp().and(m_joystick.leftBumper().negate()).onTrue(new L4(m_elevator, m_windmill));
        m_joystick.povUp().and(m_joystick.leftBumper().negate())
            .onFalse(new CGOuttakeThenStow(ManipulatorCalibrations.kL4OuttakeSpeed, 
                                                         ManipulatorCalibrations.kL4OuttakeTime, 
                                                         m_elevator, m_windmill, m_manipulator));

        /* Coral reef L3 dropoff sequence */
        m_joystick.povLeft().and(m_joystick.leftBumper().negate()).onTrue(new L3(m_elevator, m_windmill));
        m_joystick.povLeft().and(m_joystick.leftBumper().negate())
            .onFalse(new CGOuttakeThenStow(
                ManipulatorCalibrations.kL3OuttakeSpeed,
                ManipulatorCalibrations.kL3OuttakeTime,
                m_elevator, m_windmill, m_manipulator));

        /* Coral reef L2 dropoff sequence */
        m_joystick.povRight().and(m_joystick.leftBumper().negate()).onTrue(new L2(m_elevator, m_windmill));
        m_joystick.povRight().and(m_joystick.leftBumper().negate())
            .onFalse(new CGOuttakeThenStow(
                ManipulatorCalibrations.kL2OuttakeSpeed, ManipulatorCalibrations.kL2OuttakeTime, 
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

        m_joystick.y().onTrue(new PrepClimb(m_elevator, m_windmill)).onFalse(new CGClimb(m_windmill, m_elevator));

        m_joystick.a().onTrue(new AlgaeFloorPickup(m_elevator, m_windmill, m_manipulator));

        m_joystick.povRight().and(m_joystick.leftBumper()).onTrue(new AlgaeL2Pickup(m_elevator, m_windmill, m_manipulator));

        m_joystick.povLeft().and(m_joystick.leftBumper()).onTrue(new AlgaeL3Pickup(m_elevator, m_windmill, m_manipulator));

        m_joystick.povDown().and(m_joystick.leftBumper()).onTrue(new ProcessAlgae(m_elevator, m_windmill));
        m_joystick.povDown().and(m_joystick.leftBumper()).onFalse(new RunManipulator(
            ManipulatorCalibrations.kAlgaeBargingVelocity, 
            ManipulatorCalibrations.kMaxAcceleration, 
            m_manipulator).withTimeout(1));

        m_joystick.back().onTrue(new BargeAlgae(m_elevator, m_windmill))
            .onFalse(new RunManipulator(
                ManipulatorCalibrations.kAlgaeBargingVelocity, 
                ManipulatorCalibrations.kMaxAcceleration, 
                m_manipulator)
            .withTimeout(1));

    }


    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
