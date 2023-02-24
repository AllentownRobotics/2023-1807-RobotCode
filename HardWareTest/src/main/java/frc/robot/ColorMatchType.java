package frc.robot;

public enum ColorMatchType{
    CubeMatch("Cube"),
    ConeMatch("Cone"),
    NullMatch("Null");

    public final String asString;

    ColorMatchType(String asString){
        this.asString = asString;
    }
}