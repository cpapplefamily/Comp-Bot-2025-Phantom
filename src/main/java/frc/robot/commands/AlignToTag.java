// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.DriverCalibrations;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;


/**
 * AlignToTag command.
 */
public class AlignToTag extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
    private FieldCentric m_swerveRequest;
    private DoubleSupplier m_driverX;
    private DoubleSupplier m_driverY;
    private double m_rotationSpeed;
    private int m_pipeline;

    /**
     * AlignToTag Constructor.
     *
     * @param driverX The driver input for X translation as a value from [0, 1]
     * @param driverY The driver input for Y translation as a value from [0, 1]
     * @param drivetrain The drivetrain
     */
    public AlignToTag(int pipeline, DoubleSupplier driverX, DoubleSupplier driverY, CommandSwerveDrivetrain drivetrain) {
        m_pipeline = pipeline;
        m_driverX = driverX;
        m_driverY = driverY;
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex("limelight-one", m_pipeline);
    }

    @Override
    public void execute() {
        m_rotationSpeed = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx")
                                              .getDouble(DriverCalibrations.kLimelightDefaultKX)
                                              * DriverCalibrations.kAprilTagAlignmentKP;
        
        m_drivetrain.setControl(m_swerveRequest.withVelocityX(m_driverX.getAsDouble() * DriverCalibrations.kmaxSpeed)
                                               .withVelocityY(m_driverY.getAsDouble() * DriverCalibrations.kmaxSpeed)
                                               .withRotationalRate(m_rotationSpeed));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
