package frc.robot.subsystems;

import frc.robot.subsystems.ElevatorStatemachine.ElevatorState;

public class ElevatorTransition {
    private final ElevatorState targetState;

    public ElevatorTransition(ElevatorState targetState) {
        this.targetState = targetState;
    }

    public boolean isTriggered() {
        return false;
    }

    public void performTransitionAction() {
    }

    public boolean isSuccess() {
        return false;
    }

    public ElevatorState getNextState() {
        return targetState;
    }

    public boolean isExpired(){
        return false;
    }

    public void startTimer(){

    }
}
