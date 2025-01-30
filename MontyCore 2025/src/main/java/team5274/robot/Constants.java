// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5274.robot;

/** Add your docs here. */
public class Constants {

    public static class ElevatorConstants {
        public static final double kGearRatio = 0.0; //Configure

        //Extrema
        public static final double kMinRotations = 0.0;
        public static final double kMaxRotations = 0.0; //Configure
        public static final double kMinHeight = 0.0;
        public static final double kMaxHeight = 0.0; //Configure
        public static final double kHomingHeight = kMinHeight;

        //Tolerances
        public static final double kRotationTolerance = 0.1;
        public static final double kHeightTolerance = 0.5;

        //Placement
        public static final double kTroughHeight = 0.0; //Configure
        public static final double kL1Height = 0.0; //Configure
        public static final double kL2Height = 0.0; //Configure
        public static final double kL3Height = 0.0; //Configure

        //Intaking
        public static final double kStationHeight = 0.0; //Configure
        public static final double kHandoffHeight = 0.0; //Configure
    }

}