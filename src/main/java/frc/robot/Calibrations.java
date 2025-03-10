package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

/**
 * Robot calibrations.
 */
public class Calibrations {

    /**
     * Driver calibrations.
     */
    public static class DriverCalibrations {

        /* Max speed, in meters per seocond, of the robot */
        public static final double kmaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        
        /* 3/4 max angular velocity, in rotations per second, of the robot */
        public static final double kmaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /* P-gain for rotational controller using LL tx as feedback */
        public static final double kAprilTagRotationAlignmentKP = 0.05;
        
        /* When the LL doesn't see a tag, use this value...which essentially sets the error to 0 */
        public static final double kLimelightDefaultKTx = 0;

        /* P-gain for robot-centric X-translational controller using LL tx as feedback */
        public static final double kAprilTagTranslationXAlignmentKP = 0.07;
        
        /* Robot-centric X-translational controller - add a little Y-translation to stay flush to the coral reef */
        public static final double kAprilTagTranslationYRate = -0.8;
        
    }

    /**
     * Elevator calibrations.
     */
    public static class ElevatorCalibrations {

        /* Gains for the MotionMagic profiler for teleoperated elevator */
        public static final double kG = 5;
        public static final double kS = 2.5;
        public static final double kA = 0;
        public static final double kV = 0;
        public static final double kP = 4;
        public static final double kD = 0.6;

        /* Gains for the MotionMagic profiler for endgame elevator */
        public static final double kClimbG = -25;

        /* MotionMagic constraints for teleoperated elevator */
        public static final double kMaxSpeedMotionMagic = 500;
        public static final double kMaxAccelerationMotionMagic = 600;
        public static final double kMaxCurrentPerMotor = 80;

        /* MotionMagic constraints for engame elevator */
        public static final double kClimbSpeedMotionMagic = 18;

        /* Soft limit in rotor rotations */
        public static final double kForwardSoftLimitThreshold = 88;

        /* atTarget() returns true if within this tolerance, in inches */
        public static final double kDefaultTolerance = 5;

        /* Bottom position in inches */
        public static final double kBottomPosition = -0.2;

        /* Pendulum position in inches */
        public static final double kPendulumPosition = 28;

        /* Pendulum tolerance in inches before any windmill movement */
        public static final double kPendulumTolerance = 2;

        /* Coral station position in inches */
        public static final double kCoralStationPosition = 34.5;

        /* Coral station tolerance in inches before any windmill movement */
        public static final double kCoralStationTolerance = 2;

        /* Coral reef L4 position in inches */
        public static final double kL4Position = 52.5;

        /* Coral reef L4 tolerance in inches before any windmill movement */
        public static final double kL4Tolerance = 2;

        /* Coral reef L3 position in inches */
        public static final double kL3Position = 24;

        /* Coral reef L3 tolerance in inches before any windmill movement */
        public static final double kL3Tolerance = 2;

        /* Coral reef L2 position in inches */
        public static final double kL2Position = 8.5;

        /* Coral reef L2 tolerance in inches before any windmill movement */
        public static final double kL2Tolerance = 2;

        /* Coral station stow position in inches, windmilll moves after elevator */
        public static final double kCoralStationStowPosition = 30;

        /* Climb position in inches */
        public static final double kClimbPosition = 4;

        /* Barge position in inches */
        public static final double kBargePosition = 53.5;

        /* Servo lock position in degrees */
        public static final int kservoLockAngle = 100;

        /* Servo unlock position in degrees */
        public static final int kservoUnlockAngle = 30;

    }

    /**
     * Windmill calibrations.
     */
    public static class WindmillCalibrations {
        
        /* The encoder offset when the windmill is in lollipop position */
        public static final double kCanCoderOffset = 0.76708984375;

        /* Gains for the MotionMagic profiler for teleoperated */
        public static final double kG = 10.6;
        public static final double kS = 3.0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kP = 800;
        public static final double kD = 200;

        /* MotionMagic constraints for teleoperated */
        public static final double kMaxSpeedMotionMagic = 1;
        public static final double kMaxAccelerationMotionMagic = 4;
        public static final double kMaxJerkMotionMagic = 100;
        public static final double kMaxWindmillStatorCurrentPerMotor = 80;

        /* MotionMagic constraints for endgame */
        public static final double kClimbMaxSpeedMotionMagic = 0.25;

        /* atTarget() returns true if within this tolerance, in degrees */
        public static final double kWindmillTolerance = 10;

        /* Lollipop position in degrees */
        public static final double kLollipopPosition = 90;

        /* Lollipop tolerance in degrees before any elevator movement */
        public static final double kLollipopTolerance = 30;

        /* Pendulum position in degrees */
        public static final double kPendulumPosition = 270;
        
        /* Coral station position in degrees */
        public static final double kCoralStationPosition = 287;
        
        /* Coral reef L4 position in degrees */
        public static final double kL4Position = 120;

        /* Coral reef L3 position in degrees */
        public static final double kL3Position = 110;

        /* Coral reef L2 position in degrees */
        public static final double kL2Position = 110;

        /* Coral station position in degrees */
        public static final double kCoralStationStowPosition = 287;
        
        /* Coral station tolerance in degrees before any elevator movement */
        public static final double kCoralStationStowTolerance = 5;

        /* Climb position in degrees */
        public static final double kClimbPosition = 120;

        /* Climb tolerance in degrees before any elevator movement */
        public static final double kClimbTolerance = 5;

        /* Barge position in degrees */
        public static final double kBargePosition = 90;

        /* Barge tolerance in degrees before any elevator movement */
        public static final double kBargeTolerance = 10;
        
    }

    /**
     * Manipulator Calibrations.
     */
    public static class ManipulatorCalibrations {
        
        /* Gains for the MotionMagic velocity controller */
        public static final double kS = 6;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kP = 5;
        public static final double kD = 0;

        /* MotionMagic constraints for velocity controller */
        public static final double kMaxAcceleration = 1000;
        public static final double kMaxSpeed = 100;
        public static final double kMaxStatorCurrent = 40;
 
        /* Stator current delta threshold to stop motors */
        public static final double kCurrentThreshold = 15;

        /* Coral reef L4 outtake speed, in rotations per second */
        public static final double kL4OuttakeSpeed = -30;

        /* Coral reef L4 outtake time, in seconds */
        public static final double kL4OuttakeTime = 0.25;

        /* Coral reef L3 outtake speed, in rotations per second */
        public static final double kL3OuttakeSpeed = -30;

        /* Coral reef L3 outtake time, in seconds */
        public static final double kL3OuttakeTime = 0.25;

        /* Coral reef L2 outtake speed, in rotations per second */
        public static final double kL2OuttakeSpeed = -30;

        /* Coral reef L2 outtake time, in seconds */
        public static final double kL2OuttakeTime = 0.25;

        /* Algae intake and holding speed, in rotations per second */
        public static final double kAlgaeHoldingVelocity = -5;

        /* Algae barge outtake speed, in rotations per second */
        public static final double kAlgaeBargingVelocity = 100;

    }
}