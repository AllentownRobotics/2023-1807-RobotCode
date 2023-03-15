package frc.robot.Utils.Enums;

public enum DistanceState {
    TooClose,
    TooFar,
    Grabbable;

    public static DistanceState fromBools(boolean aboveLower, boolean belowUpper){
        if (!belowUpper){
            return TooFar;
        }
        if (!aboveLower){
            return TooClose;
        }
        return Grabbable;
    }
}
