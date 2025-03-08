package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import frc.robot.generated.TunerConstants;

/**
 * Robot calibrations.
 */
public class Calibrations {

    /**
     * Driver calibrations.
     */
    public static class DriverCalibrations {

        // kSpeedAt12Volts desired top speed
        public static final double kmaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // 3/4 of a rotation per second max angular velocity
        public static final double kmaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        public static final double kMaxAngularAcceleration = RotationsPerSecond.per(Second).of(0.75)
                                                                               .in(RadiansPerSecondPerSecond);

        public static final double kAprilTagAlignmentKP = 1;
        public static final double kAprilTagAlignmentKI = 0;
        public static final double kAprilTagAlignmentKD = 0;

        public static final double kLimelightDefaultKX = 0;
        
    }

    /**
     * Elevator calibrations.
     */
    public static class ElevatorCalibrations {

        //All of the PID and Feedforward gains for the MotionMagic Motion profiler.
        public static final double kG = 5;
        public static final double kS = 2.5;
        public static final double kA = 0;
        public static final double kV = 0;
        public static final double kP = 4;
        public static final double kD = 0.6;

        // public static final double kElevatorClimbkP = 24;
        // public static final double kElevatorClimbkD = 10;
        public static final double kClimbG = -25;
        public static final double kClimbSpeedMotionMagic = 18;


        // Motion Magic Configs for the MotionMagicConfigs class for the Elevator
        public static final double kMaxSpeedMotionMagic = 500;
        public static final double kMaxAccelerationMotionMagic = 600;
        public static final double kMaxCurrentPerMotor = 80;

        // Swith limits
        public static final double kForwardSoftLimitThreshold = 88;

        public static final double kBottomPosition = -0.2;

        public static final double kPendulumTolerance = 2;
        public static final double kPendulumPosition = 28;

        public static final double kCoralStationTolerance = 2;
        public static final double kCoralStationPosition = 34.5;

        public static final double kL4Tolerance = 2;
        public static final double kL4Position = 52.5;

        public static final double kL3Tolerance = 2;
        public static final double kL3Position = 24;

        public static final double kL2Tolerance = 2;
        public static final double kL2Position = 8.5;

    }

    /**
     * Windmill calibrations.
     */
    public static class WindmillCalibrations {
        
        // The encoder offset for the windmill
        public static final double kCanCoderOffset = 0.76708984375;

        // All of the PID and Feedforward gains for the MotionMagic Motion profiler.
        public static final double kG = 10.6;
        public static final double kS = 3.0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kP = 800;
        public static final double kD = 200;

        // Motion Magic Configs for the MotionMagicConfigs Class for the Windmill
        public static final double kMaxSpeedMotionMagic = 1;
        public static final double kMaxAccelerationMotionMagic = 4;
        public static final double kMaxJerkMotionMagic = 100;

        public static final double kMaxWindmillStatorCurrentPerMotor = 80;

        public static final double kClimbMaxSpeedMotionMagic = 0.25;

        // The windmill will report that it is at it's setpoint if it is within this amount of degrees.
        public static final double kWindmillTolerance = 5;

        public static final double kLollipopTolerance = 30;
        public static final double kLollipopPosition = 90;

        public static final double kPendulumPosition = 270;
        
        public static final double kCoralStationPosition = 287;
        public static final double kL4Position = 120;
        public static final double kL3Position = 110;
        public static final double kL2Position = 110;
    }

    /**
     * Manipulator Calibrations.
     */
    public static class ManipulatorCalibrations {
        
        public static final double kS = 6;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kP = 5;
        public static final double kD = 0;

        public static final double kMaxAcceleration = 140;
        public static final double kMaxSpeed = 100;
        public static final double kMaxStatorCurrent = 40;
        public static final double kCurrentThreshold = 15;

        public static final double kL4OuttakeSpeed = -30;
        public static final double kL4OuttakeTime = 0.25;

        public static final double kL3OuttakeSpeed = -30;
        public static final double kL3OuttakeTime = 0.25;

        public static final double kL2OuttakeSpeed = -30;
        public static final double kL2OuttakeTime = 0.25;

    }
}