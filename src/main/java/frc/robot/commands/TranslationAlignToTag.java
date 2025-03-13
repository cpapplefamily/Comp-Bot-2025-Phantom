// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.DriverCalibrations;
import frc.robot.subsystems.CommandSwerveDrivetrain;


/**
 * AlignToTag command.
 */
public class TranslationAlignToTag extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
    private double m_xspeed;
    private int m_pipeline;

    private RobotCentric m_swerveRequest = new RobotCentric().withRotationalDeadband(DriverCalibrations.kmaxSpeed * 0.1);

    /**
     * AlignToTag Constructor.
     *
     * @param drivetrain The drivetrain
     */
    public TranslationAlignToTag(int pipeline, CommandSwerveDrivetrain drivetrain) {
        m_pipeline = pipeline;
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("pipeline").setDouble(m_pipeline);
    }

    @Override
    public void execute() {
        m_xspeed = NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("tx")
                                              .getDouble(DriverCalibrations.kLimelightDefaultKTx)
                                              * DriverCalibrations.kAprilTagTranslationXAlignmentKP;
        

        m_drivetrain.setControl(m_swerveRequest
            .withVelocityX(m_xspeed)
            .withVelocityY(DriverCalibrations.kAprilTagTranslationYRate));

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
