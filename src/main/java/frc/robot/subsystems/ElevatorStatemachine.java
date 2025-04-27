package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorStatemachine.ElevatorState;

public class ElevatorStatemachine extends SubsystemBase {

    // Define the states for the elevator
    protected enum ElevatorState {
        INITIAL,
        HOME_UP,
        HOME_DOWN,
        HOMED,
        L1,
        L2,
        L3,
        L4
    }

    private ElevatorState currentState;

    private boolean isStateUpdating = false;

    private final Map<ElevatorState, List<ElevatorTransition>> transitionMap = new HashMap<>();

    protected ElevatorSubsystem elevator;

    public ElevatorStatemachine() {
        currentState = ElevatorState.INITIAL;
        for (ElevatorState state : ElevatorState.values()) {
            transitionMap.putIfAbsent(state, new ArrayList<>());
        }
        transitionMap.get(ElevatorState.INITIAL).add(new Transition05(elevator));
        transitionMap.get(ElevatorState.INITIAL).add(new Transition06(elevator));
        transitionMap.get(ElevatorState.HOME_UP).add(new Transition56(elevator));
        transitionMap.get(ElevatorState.HOME_DOWN).add(new Transition67(elevator));
        transitionMap.get(ElevatorState.HOMED).add(new Transition71(elevator));
        transitionMap.get(ElevatorState.HOMED).add(new Transition72(elevator));
        transitionMap.get(ElevatorState.HOMED).add(new Transition73(elevator));
        transitionMap.get(ElevatorState.HOMED).add(new Transition74(elevator));
        transitionMap.get(ElevatorState.L1).add(new Transition17(elevator));
        transitionMap.get(ElevatorState.L2).add(new Transition27(elevator));
        transitionMap.get(ElevatorState.L3).add(new Transition37(elevator));
        transitionMap.get(ElevatorState.L4).add(new Transition47(elevator));
    }

    public ElevatorState getCurrentState() {
        return currentState;
    }

    public void setState(ElevatorState newState) {
        currentState = newState;
    }

    public void update() {
        this.isStateUpdating = false;
        List<ElevatorTransition> transitions = transitionMap.getOrDefault(currentState, List.of());
        for (ElevatorTransition t : transitions) {
            if (t.isTriggered()) {
                this.isStateUpdating = true;
                t.startTimer();
                t.performTransitionAction();

                if (t.isSuccess()) {
                    this.currentState = t.getNextState();
                }
                if (t.isExpired()) {
                    this.currentState = t.getNextState();
                }
            }
        }
    }

    @Override
    public void periodic() {
    update();
    if(!isStateUpdating){
        switch (currentState) {
            case INITIAL:
                elevator.setNeutral();
                break;
            
            case HOME_UP:
                elevator.setNeutral();
                break;

            case HOME_DOWN:
                elevator.setNeutral();
                break;

            case HOMED:
                elevator.setNeutral();
                break;

            case L1:
                elevator.setPosition(ElevatorConstants.L1Position);
                break;

            case L2:
                elevator.setPosition(ElevatorConstants.L2Position);
                break;
            
            case L3:
                elevator.setPosition(ElevatorConstants.L3Position);
                break;
            
            case L4:
                elevator.setPosition(ElevatorConstants.L4Position);
                break;
            default:
                elevator.setNeutral();
                break;
            }
        }
    }
}

class Transition05 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition05(ElevatorSubsystem elevator) {
        super(ElevatorState.INITIAL);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return elevator.isHallSensorTriggered() && elevator.getHomeRequest();
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void performTransitionAction() {
        elevator.setVoltage(ElevatorConstants.home_upVoltage);
    }

    @Override
    public boolean isSuccess() {
        return !elevator.isHallSensorTriggered();
    }

    @Override
    public boolean isExpired() {
        return (Timer.getFPGATimestamp() - startTime) > ElevatorConstants.home_upTime;
    }

    @Override
    public ElevatorState getNextState() {
        if (isSuccess()) {
            return ElevatorState.HOME_DOWN;
        } else {
            return ElevatorState.INITIAL;
        }
    }
}

class Transition06 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition06(ElevatorSubsystem elevator) {
        super(ElevatorState.INITIAL);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return !elevator.isHallSensorTriggered() && elevator.getHomeRequest();
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void performTransitionAction() {

    }

    @Override
    public boolean isSuccess() {
        return true;
    }

    @Override
    public boolean isExpired() {
        return true;
    }

    @Override
    public ElevatorState getNextState() {
        return ElevatorState.HOME_DOWN;
    }
}

class Transition56 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition56(ElevatorSubsystem elevator) {
        super(ElevatorState.HOME_UP);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return true;
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void performTransitionAction() {

    }

    @Override
    public boolean isSuccess() {
        return true;
    }

    @Override
    public boolean isExpired() {
        return true;
    }

    @Override
    public ElevatorState getNextState() {
        return ElevatorState.HOME_DOWN;
    }
}

class Transition67 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition67(ElevatorSubsystem elevator) {
        super(ElevatorState.HOME_DOWN);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return elevator.getHomeRequest();
    }

    @Override
    public void performTransitionAction() {
        elevator.setVoltage(ElevatorConstants.home_downVoltage);
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isSuccess() {
        return elevator.isHallSensorTriggered();
    }

    @Override
    public boolean isExpired() {
        return (Timer.getFPGATimestamp() - startTime) > ElevatorConstants.home_downTime;
    }

    @Override
    public ElevatorState getNextState() {
        if (isSuccess()) {
            return ElevatorState.HOMED;
        } else {
            return ElevatorState.INITIAL;
        }
    }
}

class Transition71 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition71(ElevatorSubsystem elevator) {
        super(ElevatorState.HOMED);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return elevator.getL1Request();
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void performTransitionAction() {
        elevator.setPosition(ElevatorConstants.L1Position);
    }

    @Override
    public boolean isSuccess() {
        return elevator.isAtPosition(ElevatorConstants.L1Position);
    }

    @Override
    public boolean isExpired() {
        return false;
    }

    @Override
    public ElevatorState getNextState() {
        if (isSuccess()) {
            return ElevatorState.L1;
        } else {
            return ElevatorState.HOMED;
        }
    }
}

class Transition72 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition72(ElevatorSubsystem elevator) {
        super(ElevatorState.HOMED);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return elevator.getL2Request();
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void performTransitionAction() {
        elevator.setPosition(ElevatorConstants.L2Position);
    }

    @Override
    public boolean isSuccess() {
        return elevator.isAtPosition(ElevatorConstants.L2Position);
    }

    @Override
    public boolean isExpired() {
        return false;
    }

    @Override
    public ElevatorState getNextState() {
        if (isSuccess()) {
            return ElevatorState.L2;
        } else {
            return ElevatorState.HOMED;
        }
    }
}

class Transition73 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition73(ElevatorSubsystem elevator) {
        super(ElevatorState.HOMED);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return elevator.getL3Request();
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void performTransitionAction() {
        elevator.setPosition(ElevatorConstants.L3Position);
    }

    @Override
    public boolean isSuccess() {
        return elevator.isAtPosition(ElevatorConstants.L3Position);
    }

    @Override
    public boolean isExpired() {
        return false;
    }

    @Override
    public ElevatorState getNextState() {
        if (isSuccess()) {
            return ElevatorState.L3;
        } else {
            return ElevatorState.HOMED;
        }
    }
}

class Transition74 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition74(ElevatorSubsystem elevator) {
        super(ElevatorState.HOMED);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return elevator.getL4Request();
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void performTransitionAction() {
        elevator.setPosition(ElevatorConstants.L4Position);
    }

    @Override
    public boolean isSuccess() {
        return elevator.isAtPosition(ElevatorConstants.L4Position);
    }

    @Override
    public boolean isExpired() {
        return false;
    }

    @Override
    public ElevatorState getNextState() {
        if (isSuccess()) {
            return ElevatorState.L4;
        } else {
            return ElevatorState.HOMED;
        }
    }
}

class Transition17 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition17(ElevatorSubsystem elevator) {
        super(ElevatorState.L1);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return !elevator.getL1Request();
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void performTransitionAction() {
        elevator.setPosition(ElevatorConstants.homePosition);
    }

    @Override
    public boolean isSuccess() {
        return elevator.isAtPosition(ElevatorConstants.homePosition);
    }

    @Override
    public boolean isExpired() {
        return (Timer.getFPGATimestamp() - startTime) > ElevatorConstants.reset2HomeTime;
    }

    @Override
    public ElevatorState getNextState() {
        if (isSuccess()) {
            return ElevatorState.HOMED;
        } else {
            return ElevatorState.INITIAL;
        }
    }
}

class Transition27 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition27(ElevatorSubsystem elevator) {
        super(ElevatorState.L2);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return !elevator.getL2Request();
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void performTransitionAction() {
        elevator.setPosition(ElevatorConstants.homePosition);
    }

    @Override
    public boolean isSuccess() {
        return elevator.isAtPosition(ElevatorConstants.homePosition);
    }

    @Override
    public boolean isExpired() {
        return (Timer.getFPGATimestamp() - startTime) > ElevatorConstants.reset2HomeTime;
    }

    @Override
    public ElevatorState getNextState() {
        if (isSuccess()) {
            return ElevatorState.HOMED;
        } else {
            return ElevatorState.INITIAL;
        }
    }
}

class Transition37 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition37(ElevatorSubsystem elevator) {
        super(ElevatorState.L3);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return !elevator.getL3Request();
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void performTransitionAction() {
        elevator.setPosition(ElevatorConstants.homePosition);
    }

    @Override
    public boolean isSuccess() {
        return elevator.isAtPosition(ElevatorConstants.homePosition);
    }

    @Override
    public boolean isExpired() {
        return (Timer.getFPGATimestamp() - startTime) > ElevatorConstants.reset2HomeTime;
    }

    @Override
    public ElevatorState getNextState() {
        if (isSuccess()) {
            return ElevatorState.HOMED;
        } else {
            return ElevatorState.INITIAL;
        }
    }
}

class Transition47 extends ElevatorTransition {

    private ElevatorSubsystem elevator;
    private double startTime = -1.0;

    public Transition47(ElevatorSubsystem elevator) {
        super(ElevatorState.L4);
        this.elevator = elevator;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isTriggered() {
        return !elevator.getL4Request();
    }

    @Override
    public void startTimer() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void performTransitionAction() {
        elevator.setPosition(ElevatorConstants.homePosition);
    }

    @Override
    public boolean isSuccess() {
        return elevator.isAtPosition(ElevatorConstants.homePosition);
    }

    @Override
    public boolean isExpired() {
        return (Timer.getFPGATimestamp() - startTime) > ElevatorConstants.reset2HomeTime;
    }

    @Override
    public ElevatorState getNextState() {
        if (isSuccess()) {
            return ElevatorState.HOMED;
        } else {
            return ElevatorState.INITIAL;
        }
    }
}