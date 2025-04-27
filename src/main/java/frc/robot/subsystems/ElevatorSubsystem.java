// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX leader = new TalonFX(ElevatorConstants.kLeaderTalonID, "rio");
  private final TalonFX follower = new TalonFX(ElevatorConstants.kFollowerTalonID, "rio");
  private final DigitalInput hallSensor = new DigitalInput(ElevatorConstants.kHallSensorID);

  private TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
  private TalonFXConfiguration followerConfigs = new TalonFXConfiguration();

  private MotionMagicTorqueCurrentFOC leaderMotionMagic = new MotionMagicTorqueCurrentFOC(0.0);
  private MotionMagicTorqueCurrentFOC followerMotionMagic = new MotionMagicTorqueCurrentFOC(0.0);
  private VoltageOut leaderVoltage = new VoltageOut(0.0);
  private VoltageOut followerVoltage = new VoltageOut(0.0);
  private NeutralOut neutral = new NeutralOut();

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("elevatortable");

  private boolean homeRequest = false;
  private boolean L1Request = false;
  private boolean L2Request = false;
  private boolean L3Request = false;
  private boolean L4Request = false;

  public ElevatorSubsystem() {
    // config neutralmode
    leaderConfigs.MotorOutput.withNeutralMode(
        ElevatorConstants.leader_Neutalmode_Coast
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);
    followerConfigs.MotorOutput.withNeutralMode(
        ElevatorConstants.leader_Neutalmode_Coast
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);
    // config direction
    // POSITIVE for leader moving up and NEGATIVE for follower moving up
    leaderConfigs.MotorOutput.withInverted(
        ElevatorConstants.leader_Inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    followerConfigs.MotorOutput.withInverted(
        ElevatorConstants.follower_Inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // config PIDSAV
    leaderConfigs.Slot0.kP = ElevatorConstants.leader_kP;
    leaderConfigs.Slot0.kI = ElevatorConstants.leader_kI;
    leaderConfigs.Slot0.kD = ElevatorConstants.leader_kD;
    leaderConfigs.Slot0.kS = ElevatorConstants.leader_kS;
    leaderConfigs.Slot0.kA = ElevatorConstants.leader_kA;
    leaderConfigs.Slot0.kV = ElevatorConstants.leader_kV;
    followerConfigs.Slot0.kP = ElevatorConstants.follower_kP;
    followerConfigs.Slot0.kI = ElevatorConstants.follower_kI;
    followerConfigs.Slot0.kD = ElevatorConstants.follower_kD;
    followerConfigs.Slot0.kS = ElevatorConstants.follower_kS;
    followerConfigs.Slot0.kA = ElevatorConstants.follower_kA;
    followerConfigs.Slot0.kV = ElevatorConstants.follower_kV;

    // config motionmagic
    leaderConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.leader_CruiseVelocity;
    leaderConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.leader_CruiseAcceleration;
    followerConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.follower_CruiseVelocity;
    followerConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.follower_CruiseAcceleration;

    // config ducy cycle limit
    leaderConfigs.MotorOutput.withPeakForwardDutyCycle(ElevatorConstants.forwardDutyCycleLimit);
    leaderConfigs.MotorOutput.withPeakReverseDutyCycle(ElevatorConstants.reverseDutyCycleLimit);
    followerConfigs.MotorOutput.withPeakForwardDutyCycle(ElevatorConstants.forwardDutyCycleLimit);
    followerConfigs.MotorOutput.withPeakReverseDutyCycle(ElevatorConstants.reverseDutyCycleLimit);

    // config softlimit
    // hall sensor is used as the reverse limit
    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = ElevatorConstants.leader_forwardSoftLimitEnable;
    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.leader_forwardSoftLimitThreshold;
    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = ElevatorConstants.follower_forwardSoftLimitEnable;
    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.follower_forwardSoftLimitThreshold;
    leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ElevatorConstants.leader_reverseSoftLimitEnable;
    leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.leader_reverseSoftLimitThreshold;
    followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ElevatorConstants.follower_reverseSoftLimitEnable;
    followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.follower_reverseSoftLimitThreshold;

    follower.setControl(new Follower(ElevatorConstants.kLeaderTalonID, true));
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

  public void setPosition(double position) {
    leader.setControl(leaderMotionMagic.withPosition(position));
    follower.setControl(followerMotionMagic.withPosition(position));
  }

  public boolean isAtPosition(double position){
    return Math.abs((leader.getPosition().getValueAsDouble()+follower.getPosition().getValueAsDouble())/2 - position) < ElevatorConstants.positionDeadBand;
  }

  public void setVoltage(double voltage) {
    leader.setControl(leaderVoltage.withOutput(voltage));
    follower.setControl(followerVoltage.withOutput(voltage));
  }

  public void hold() {
    leader.setControl(leaderMotionMagic.withPosition(leader.getPosition().getValueAsDouble()));
    follower.setControl(followerMotionMagic.withPosition(follower.getPosition().getValueAsDouble()));
  }

  public void setNeutral() {
    leader.setControl(neutral);
    follower.setControl(neutral);
  }

  public boolean isHallSensorTriggered() {
    return hallSensor.get();
  }

  public void upDateNetworkTable(){
    table.getBooleanTopic("elevator/hallSensor").publish().set(isHallSensorTriggered());
  }

  public void setHomeRequest(boolean homeRequest){
    this.homeRequest = homeRequest;
  }

  public boolean getHomeRequest(){
    return homeRequest;
  }

  public void setL1Request(boolean L1Request){
    this.L1Request = L1Request;
  }

  public boolean getL1Request(){
    return L1Request;
  }

  public void setL2Request(boolean L2Request){
    this.L2Request = L2Request;
  }

  public boolean getL2Request(){
    return L2Request;
  }

  public void setL3Request(boolean L3Request){
    this.L3Request = L3Request;
  }

  public boolean getL3Request(){
    return L3Request;
  }

  public void setL4Request(boolean L4Request){
    this.L4Request = L4Request;
  }

  public boolean getL4Request(){
    return L4Request;
  }
}
