// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TwoKrakenElevatorConstants;

public class TwoKrakenElevatorSubsystem extends SubsystemBase {

  private final TalonFX leader = new TalonFX(TwoKrakenElevatorConstants.kLeaderTalonID, "rio");
  private final TalonFX follower = new TalonFX(TwoKrakenElevatorConstants.kFollowerTalonID, "rio");

  private TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
  private TalonFXConfiguration followerConfigs = new TalonFXConfiguration();

  private MotionMagicVoltage leaderMotionMagic = new MotionMagicVoltage(0.0);
  private MotionMagicTorqueCurrentFOC followerMotionMagic = new MotionMagicTorqueCurrentFOC(0.0);
  private VoltageOut leaderVoltage = new VoltageOut(0.0);
  private VoltageOut followerVoltage = new VoltageOut(0.0);
  private NeutralOut neutral = new NeutralOut();

  public TwoKrakenElevatorSubsystem() {
    // config neutralmode
    leaderConfigs.MotorOutput.withNeutralMode(
        TwoKrakenElevatorConstants.leader_Neutalmode_Coast
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);
    followerConfigs.MotorOutput.withNeutralMode(
        TwoKrakenElevatorConstants.leader_Neutalmode_Coast
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);
    // config direction
    // POSITIVE for leader moving up and NEGATIVE for follower moving up
    leaderConfigs.MotorOutput.withInverted(
        TwoKrakenElevatorConstants.leader_Inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    followerConfigs.MotorOutput.withInverted(
        TwoKrakenElevatorConstants.follower_Inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // config PIDSAV
    leaderConfigs.Slot0.kP = TwoKrakenElevatorConstants.leader_kP;
    leaderConfigs.Slot0.kI = TwoKrakenElevatorConstants.leader_kI;
    leaderConfigs.Slot0.kD = TwoKrakenElevatorConstants.leader_kD;
    leaderConfigs.Slot0.kS = TwoKrakenElevatorConstants.leader_kS;
    leaderConfigs.Slot0.kA = TwoKrakenElevatorConstants.leader_kA;
    leaderConfigs.Slot0.kV = TwoKrakenElevatorConstants.leader_kV;
    followerConfigs.Slot0.kP = TwoKrakenElevatorConstants.follower_kP;
    followerConfigs.Slot0.kI = TwoKrakenElevatorConstants.follower_kI;
    followerConfigs.Slot0.kD = TwoKrakenElevatorConstants.follower_kD;
    followerConfigs.Slot0.kS = TwoKrakenElevatorConstants.follower_kS;
    followerConfigs.Slot0.kA = TwoKrakenElevatorConstants.follower_kA;
    followerConfigs.Slot0.kV = TwoKrakenElevatorConstants.follower_kV;

    // config motionmagic
    leaderConfigs.MotionMagic.MotionMagicCruiseVelocity = TwoKrakenElevatorConstants.leader_CruiseVelocity;
    leaderConfigs.MotionMagic.MotionMagicAcceleration = TwoKrakenElevatorConstants.leader_CruiseAcceleration;
    followerConfigs.MotionMagic.MotionMagicCruiseVelocity = TwoKrakenElevatorConstants.follower_CruiseVelocity;
    followerConfigs.MotionMagic.MotionMagicAcceleration = TwoKrakenElevatorConstants.follower_CruiseAcceleration;

    // config ducy cycle limit
    leaderConfigs.MotorOutput.withPeakForwardDutyCycle(TwoKrakenElevatorConstants.forwardDutyCycleLimit);
    leaderConfigs.MotorOutput.withPeakReverseDutyCycle(TwoKrakenElevatorConstants.reverseDutyCycleLimit);
    followerConfigs.MotorOutput.withPeakForwardDutyCycle(TwoKrakenElevatorConstants.forwardDutyCycleLimit);
    followerConfigs.MotorOutput.withPeakReverseDutyCycle(TwoKrakenElevatorConstants.reverseDutyCycleLimit);

    // config softlimit
    // hall sensor is used as the reverse limit
    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = TwoKrakenElevatorConstants.leader_forwardSoftLimitEnable;
    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TwoKrakenElevatorConstants.leader_forwardSoftLimitThreshold;
    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = TwoKrakenElevatorConstants.follower_forwardSoftLimitEnable;
    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TwoKrakenElevatorConstants.follower_forwardSoftLimitThreshold;
    leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = TwoKrakenElevatorConstants.leader_reverseSoftLimitEnable;
    leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TwoKrakenElevatorConstants.leader_reverseSoftLimitThreshold;
    followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = TwoKrakenElevatorConstants.follower_reverseSoftLimitEnable;
    followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TwoKrakenElevatorConstants.follower_reverseSoftLimitThreshold;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setLeaderPosition(double position){
    leader.setControl(leaderMotionMagic.withPosition(position));
  }

  public void setFollowerPosition(double position){
    follower.setControl(followerMotionMagic.withPosition(position));
  }

  public void setLeaderVoltage(double voltage){
    leader.setControl(leaderVoltage.withOutput(voltage));
  }

  public void setFollowerVoltage(double voltage){
    follower.setControl(followerVoltage.withOutput(voltage));
  }

  public void setNeutral(){
    leader.setControl(neutral);
    follower.setControl(neutral);
  }
}
