// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorLockConstants;

/**
 * Elevator lock subsystem.
 */
public class ElevatorLockSubsystem extends SubsystemBase {

    private final Servo m_lockServo;

    /**
     * Elevator subsystem constructor.
     */
    public ElevatorLockSubsystem() {

        /* Create the hardware */
        m_lockServo = new Servo(ElevatorLockConstants.kservoPort);
    }

    /**
     * Disable lock servo.
     */
    public void disableServo() {
        m_lockServo.setDisabled();
    }
  
    /**
     * Set the lock servo angle.
     *
     * @param angle servo angle in degrees
     */
    public void setAngle(double angle) {
        m_lockServo.setAngle(angle);
    }
  
    /**
     * Return the lock servo postion.
     *
     * @return servo position in degrees
     */
    public double getServoPos() {
        return m_lockServo.getAngle();
    }


    @Override
    public void periodic() {

    }
 
}