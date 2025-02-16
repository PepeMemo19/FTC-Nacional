package org.firstinspires.ftc.teamcode.common;

@com.acmerobotics.dashboard.config.Config
public class Config {
    //public static double pivot_kP = 0.03, pivot_kI = 0.0, pivot_kD = 0.001, pivot_n_kF = -0., pivot_h_kF = -0.06, pivot_maxH_kF = -0.2, pivot_tolerance = 5.0;
    public static double pivot_kP = 0.05, pivot_kI = 0.0, pivot_kD = 0.000, pivot_n_kF = -0.000, pivot_hN_kF = -0.0000, pivot_h_kF = -0.0068, pivot_maxH_kF = -0., pivot_tolerance = 8.0;
    //0.0009
    public static double ascent_kP = 0.01, ascent_kI = 0.0, ascent_kD = 0.0, ascent_kF = 0.0, ascent_tolerance = 10.0;
    public  static double heading_kP = 0.07, heading_kI = 0.0, heading_kD = 0.0, heading_kF = 0.0, heading_tolerance = 5.0;
    public static double ext_increment = 50.0, pivot_increment = 10.0;
    public static double extension_kP = 0.01, extension_kI = 0.0, extension_kD = 0.0, extension_kF = 0.0003, extension_tolerance = 30.0;
    public static double claw_kP = 0.007, claw_kI = 0.0, claw_kD = 0.0, claw_kF = 0.003, claw_tolerance = 2.0, claw_init = 0.0;

}