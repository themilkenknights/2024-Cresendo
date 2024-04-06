package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public final class Constants {

        public static final double stickDeadband = 0.1;

        // CANIDs and portsfor intakes
        public static final int ElevatorLeftCANID = 20;
        public static final int ElevatorRightCANID = 21;
        public static final int topIntakeCANID = 22;
        public static final int midIntakeCANID = 27;
        public static final int bottomIntakeIntakeCANID = 23;

        public static final int backIRPORT = 0;
        public static final int frontIRPORT = 1;
        public static final int ledPORT = 2;
        // public static final int blinkenPWMPORT = 3;

        // CANIDs and ports for Climb
        public static final int ClimbCANID = 24;

        public static final int ClimbServoPORT = 0;

        // current limits

        public static double DriveLimitAmps = 80;
        public static double TurnLimitAmps = 60;

        public static double ElevatorLimitAmps = 50;
        public static double IntakeLimitAmps = 80;

       public  static class limits {

                public static CurrentLimitsConfigs DriveLimits = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(DriveLimitAmps)
                                .withStatorCurrentLimitEnable(true);
                public static CurrentLimitsConfigs TurnLimits = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(TurnLimitAmps)
                                .withStatorCurrentLimitEnable(true);
                public static CurrentLimitsConfigs ElevatorLimits = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(ElevatorLimitAmps)
                                .withStatorCurrentLimitEnable(true);
                public static CurrentLimitsConfigs IntakeLimits = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(IntakeLimitAmps)
                                .withStatorCurrentLimitEnable(true);
        }

}
