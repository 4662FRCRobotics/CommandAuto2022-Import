// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveNormSubsystemConst {
        public static final int kLEFT_MOTOR1 = 22;
        public static final int kRIGHT_MOTOR1 = 23;
        public static final boolean kINVERTLEFT = false;
        public static final boolean kINVERTRIGHT = true;

        public static final double kRAMP_RATE = 1.0;
        public static final int kCURRENT_LIMT = 40;
        public static final double kMAX_VOLTAGE = 10;

        public static final double kTIRE_SIZE_IN = 6.0;
        public static final double kTIRE_SIZE_M = Units.inchesToMeters(kTIRE_SIZE_IN);
        public static final int kPULSE_PER_ROTATION = 1;

        private static final double kGEAR_1_DRIVER = 13;
        private static final double kGEAR_1_OUTPUT = 58;
        private static final double kGEAR_2_DRIVER = 8; //16; for neo - 8 is for the 500 testing
        private static final double kGEAR_2_OUTPUT = 32;
        public static final double kGEAR_REDUCTION = (kGEAR_1_OUTPUT / kGEAR_1_DRIVER) * (kGEAR_2_OUTPUT / kGEAR_2_DRIVER);
        public static final double kENCODER_DISTANCE_PER_PULSE_M = ((double) kPULSE_PER_ROTATION   / kGEAR_REDUCTION) 
             * (kTIRE_SIZE_M * Math.PI);
        public static final double kTRACK_WIDTH_M = 0.60;
        public static final DifferentialDriveKinematics kDRIVE_KINEMATICS = 
            new DifferentialDriveKinematics(kTRACK_WIDTH_M);
        
        public static final double kCONTROL_P = .133;
        public static final double kCONTROL_I = 0;
        public static final double kCONTROL_D = 2.7;
        public static final double kCONTROL_IZONE = 0;
        public static final double kCONTROL_FF = 0;
        public static final double kCONTROL_MAX_OUT = 1.0;
        public static final double kCONTROL_MIN_OUT = -1.0;

        // these should come from SYSID tool robot characterization
        public static final double kS_VOLTS = 0.0734;
        public static final double kV_VOLT_SECOND_PER_METER = 2.7;
        public static final double kA_VOLT_SEONDS_SQUARED_PER_METER = 0.074;
        public static final double kMAX_SPEED_METERS_PER_SECOND = 4;
        public static final double kMAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;

        public static final double kSMARTMOT_P = .133;
        public static final double kSMARTMOT_I = 0;
        public static final double kSMARTMOT_D = 2.7;
        public static final double kSMARTMOT_IZONE = 0;
        public static final double kSMARTMOT_FF = 0;
        public static final double kSMARTMOT_MAX_OUT = 1.0;
        public static final double kSMARTMOT_MIN_OUT = -1.0;
        public static final double kSMARTMOT_MAX_V = 500;
        public static final double kSMARTMOT_MIN_V = 0;
        public static final double kSMARTMOT_MAX_ACC = 100;
        public static final double kSMARTMOT_ALLOWED_ERR = 0.0;

        public static final double kRAMSETE_P = 0.133;
        public static final double kRAMSETE_I = 0;
        public static final double kRAMSETE_D = 2.7;
        public static final double kRAMSETE_IZONE = 0;
        public static final double kRAMSETE_FF = 0;
        public static final double kRAMSETE_MAX_OUT = 1.0;
        public static final double kRAMSETE_MIN_OUT = -1.0;
    }

    public static final class ControllerConst {
        public static final int kDRIVER_JOYSTICK_PORT = 0;
        public static final int kOPERATOR_GAMEPAD_PORT = 3;
        public static final int kOPERATOR_CONSOLE_PORT = 1; // should be 2 with everything plugged in
        public static final int kAUTONOMOUS_CONSOLE_PORT = 2; // should be 3 with everything plugged in
    }
}
