package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Elevator constants.
     */
    public static class ElevatorConstants {
        public static final int kmotor1CanId = 14;
        public static final int kmotor2CanId = 5;
        public static final int kmotor3CanId = 4;
        public static final int kmotor4CanId = 15;
        public static final int kcandiCanId = 55;
        public static final double kPulleyGearRatio = 1.6925;
        public static final int kservoPort = 0;
    }

    /**
     * Elevator lock constants.
     */
    public static class ElevatorLockConstants {
        public static final int kservoPort = 0;
    }


    /**
     * Windmill constants.
     */
    public static class WindmillConstants {
        public static final int kmotorCanId = 6;
        public static final int kcanCoderCanId = 6;
        public static final double ksensorToMechanismRatio = 1;
        public static final double krotorToSensorRatio = 74.4;
    }

    /**
     * Manipulator Constants.
     */
    public static class ManipulatorConstants {
        public static final int kmotorCanId = 13;
    }
}
