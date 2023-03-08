package frc.robot.utilities.state;

public enum OperatorState {
    REST(0), GAME_PIECE_PREP(1), GAME_PIECE_ENGAGE(2), GAME_PIECE_EXTEND(3), GAME_PIECE_PLACE(4);

    public final int stateId;

    private OperatorState(int stateId) {
        this.stateId = stateId;
    }
}