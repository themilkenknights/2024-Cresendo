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

        public static double DriveLimitStatorAmps =120;//80;
        public static double DriveLimitSupplyAmps =70;
        public static double TurnLimitStatorAmps = 40;//60

        public static double ElevatorLimitStatorAmps = 50;
        public static double IntakeLimitStatorAmps = 40;

       public  static class limits {

                public static CurrentLimitsConfigs DriveLimits = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(DriveLimitStatorAmps)
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(DriveLimitSupplyAmps)
                                .withStatorCurrentLimitEnable(true);
                public static CurrentLimitsConfigs TurnLimits = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(TurnLimitStatorAmps)
                                .withStatorCurrentLimitEnable(true);
                public static CurrentLimitsConfigs ElevatorLimits = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(ElevatorLimitStatorAmps)
                                .withStatorCurrentLimitEnable(true);
                public static CurrentLimitsConfigs IntakeLimits = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(IntakeLimitStatorAmps)
                                .withStatorCurrentLimitEnable(true);
        }

}
