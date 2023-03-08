package frc.robot.utilities.state;

public enum ScoringState {
    LOW(0), MID(1), HIGH(2), TRAVELING(3);

    public final int stateId;

    private ScoringState(int stateId) {
        this.stateId = stateId;
    }
}
