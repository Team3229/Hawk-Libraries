package frc.hawklibraries.utilities;

import lombok.Getter;
import lombok.Setter;

/**
 * Represents PID (Proportional, Integral, Derivative) constants used for tuning PID controllers.
 * This class also includes feedforward (kFF) and output range limits.
 */
@Getter
@Setter
public class PIDConstants {

  private double kP;
  private double kI;
  private double kD;
  private double kFF;
  private double kOutputMin = -1;
  private double kOutputMax = 1;

  /**
   * Constructs PIDConstants with only the proportional constant.
   *
   * @param kP Proportional constant.
   */
  public PIDConstants(double kP) {
    this.kP = kP;
  }

  /**
   * Constructs PIDConstants with proportional and integral constants.
   *
   * @param kP Proportional constant.
   * @param kI Integral constant.
   */
  public PIDConstants(double kP, double kI) {
    this.kP = kP;
    this.kI = kI;
  }

  /**
   * Constructs PIDConstants with proportional, integral, and derivative constants.
   *
   * @param kP Proportional constant.
   * @param kI Integral constant.
   * @param kD Derivative constant.
   */
  public PIDConstants(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  /**
   * Constructs PIDConstants with proportional, integral, derivative, and feedforward constants.
   *
   * @param kP Proportional constant.
   * @param kI Integral constant.
   * @param kD Derivative constant.
   * @param kFF Feedforward constant.
   */
  public PIDConstants(double kP, double kI, double kD, double kFF) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kFF = kFF;
  }

  /**
   * Constructs PIDConstants with all constants and output range limits.
   *
   * @param kP Proportional constant.
   * @param kI Integral constant.
   * @param kD Derivative constant.
   * @param kFF Feedforward constant.
   * @param kOutputMin Minimum output value.
   * @param kOutputMax Maximum output value.
   */
  public PIDConstants(
      double kP, double kI, double kD, double kFF, double kOutputMin, double kOutputMax) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kFF = kFF;
    this.kOutputMin = kOutputMin;
    this.kOutputMax = kOutputMax;
  }

  /**
   * Gets the PID constants as an array.
   *
   * @return Array of PID constants [kP, kI, kD, kFF].
   */
  public double[] getAsArray() {
    return new double[] {this.kP, this.kI, this.kD, this.kFF};
  }
}
