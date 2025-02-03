package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Params {

    // lift parameters
    public static class LiftParams {
        public static int HOME_POS    = 0;
        public static int MIDDLE_POS  = 750;
        public static int HIT_POS     = 1350;
        public static int LOW_BASKET_POS  = 3800;
        public static int CHAMBER_POS = 2150;

        public enum Position {
            // home
            HOME(HOME_POS),
            // middle
            MIDDLE(MIDDLE_POS),
            // chamber hit
            HIT(HIT_POS),
            // LOW BASKET
            LOW_BASKET(LOW_BASKET_POS),
            // chamber
            CHAMBER(CHAMBER_POS);

            Position(int pos) {
                this.position = pos;
            }

            private int position;

            public int getPos() {
                return position;
            }
        }
    }

    public static OuttakeParams LIFT_PARAMS = new OuttakeParams();


    // outtake parameters
    public static class OuttakeParams {
        // hand positions
        public double HAND_HIDE_POS = 0;
        public double HAND_HIT_POS = 0.125;
        public double HAND_HORIZONTAL_POS = 0.29;

        // claw positions
        public double CLAW_L_OPEN_POS  = 0.5,  CLAW_R_OPEN_POS = 0.75;
        public double CLAW_L_CLOSE_POS = 0.75, CLAW_R_CLOSE_POS = 0.95;
    }

    public static OuttakeParams OUTTAKE_PARAMS = new OuttakeParams();


}
