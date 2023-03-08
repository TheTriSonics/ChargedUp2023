package frc.robot.utilities.state;

public enum ScoringState {
    FLOOR(0), LOW(1), MID(2), HIGH(3), TRAVELING(4);

    public final int stateId;

    private ScoringState(int stateId) {
        this.stateId = stateId;
    }
}
