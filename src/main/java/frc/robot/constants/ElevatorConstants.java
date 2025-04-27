package frc.robot.constants;

public class ElevatorConstants {
    // initial: 0
    // homeup: 5
    // homedown: 6
    // homed: 7
    // L1: 1
    // L2: 2
    // L3: 3
    // L4: 4

    public static final int kLeaderTalonID = 10;
    public static final int kFollowerTalonID = 11;
    public static final int kHallSensorID = 12;

    public static final boolean leader_Neutalmode_Coast = true;
    public static final boolean follower_Neutalmode_Coast = true;
    public static final boolean leader_Inverted_CounterClockwisePositive = true;
    public static final boolean follower_Inverted_CounterClockwisePositive = true;

    public static final double leader_kP = 0.1;
    public static final double leader_kI = 0.0;
    public static final double leader_kD = 0.0;
    public static final double leader_kS = 0.0;
    public static final double leader_kA = 0.0;
    public static final double leader_kV = 0.0;
    public static final double follower_kP = 0.1;
    public static final double follower_kI = 0.0;
    public static final double follower_kD = 0.0;
    public static final double follower_kS = 0.0;
    public static final double follower_kA = 0.0;
    public static final double follower_kV = 0.0;

    public static final int leader_CruiseVelocity = 1000;
    public static final int leader_CruiseAcceleration = 1000;
    public static final int follower_CruiseVelocity = 1000;
    public static final int follower_CruiseAcceleration = 1000;

    public static final double forwardDutyCycleLimit = 1.0;
    public static final double reverseDutyCycleLimit = -1.0;

    public static final boolean leader_forwardSoftLimitEnable = false;
    public static final boolean leader_reverseSoftLimitEnable = false;
    public static final boolean follower_forwardSoftLimitEnable = false;
    public static final boolean follower_reverseSoftLimitEnable = false;
    public static final double leader_forwardSoftLimitThreshold = 0.0;
    public static final double leader_reverseSoftLimitThreshold = 0.0;
    public static final double follower_forwardSoftLimitThreshold = 0.0;
    public static final double follower_reverseSoftLimitThreshold = 0.0;

    public static final double home_upTime = 0.5;
    public static final double home_upVoltage = 2.0;
    public static final double home_downTime = 3.0;
    public static final double home_downVoltage = -2.0;

    public static final double L1Position = 0.0;
    public static final double L2Position = 0.0;
    public static final double L3Position = 0.0;
    public static final double L4Position = 0.0;
    public static final double homePosition = 0.0;
    public static final double reset2HomeTime = 3.0;
    public static final double positionDeadBand = 0.1;
}
