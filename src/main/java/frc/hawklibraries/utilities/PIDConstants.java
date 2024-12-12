package frc.hawklibraries.utilities;

public class PIDConstants {
    
    private double kP;
    private double kI;
    private double kD;
    private double kFF;

    public PIDConstants(double kP) {
        this.kP = kP;
    }

    public PIDConstants(double kP, double kI) {
        this.kP = kP;
        this.kI = kI;
    }

    public PIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDConstants(double kP, double kI, double kD, double kFF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
    }

    public double getkP() {return this.kP;}
    public double getkI() {return this.kI;}
    public double getkD() {return this.kD;}
    public double getkFF() {return this.kFF;}
    public double[] getAsArray() {return new double[] {
        this.kP,
        this.kI,
        this.kD,
        this.kFF
    };}

}