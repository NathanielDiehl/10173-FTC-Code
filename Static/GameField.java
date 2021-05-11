
/*
Holds the constant values of positions on the gamefield, dimensions of the robot, and color values of different field components.
Also contains the setAlliance method to switch the x-values of our tower goal positions.
 */


package org.firstinspires.ftc.teamcode.Static;

import java.util.HashMap;

public class GameField {
   public final static double robotLength = 17.75;
   public final static double robotWidth = 17.875;
   public final static double robotHeight = 0;
   public final static double wobbleGoalArmLength = 9;

   public enum Alliances {
      RED, BLUE
   }
   public static Alliances alliance = Alliances.RED;


   public final static double fieldWidth = 144;

   public final static double launchLineY = 81;

   public final static String[] goalName = {"Low", "Mid", "High", "Power 1", "Power 2", "Power 3"};
   public static double[] goalX = {30.5, 30.5, 30.5, 3.5, 11, 18.5};                             //In full field Mid x position should be negative
   public final static double[] goalY = {144, 144, 144, 144, 144, 144};
   public final static double[] goalZ = {17, 27, 35.5, 23.5, 23.5, 23.5};

   public final static double wobbleGoalDiameter = 8;
   public final static double wobbleGoalRadius = wobbleGoalDiameter / 2;
   public final static double[] wobbleGoalX = {26, 50.25};
   public final static double wobbleGoalY = 22.75;

   public final static double ringDiameter = 5;
   public final static double ringRadius = ringDiameter/2;


   public static HashMap<Character, Double> targetZoneX = new HashMap<Character, Double>() {{
      put('A', 60.125);
      put('B', 36.875);
      put('C', 60.125);
   }};
   public final static HashMap<Character, Double> targetZoneY = new HashMap<Character, Double>() {{
      put('A', 82.125);
      put('B', 106.375);
      put('C', 129.125);
   }};
   public final static double tagetZoneWidth = 24;
   public final static double[] starterStackHeight = {0.456168f, 3.149606f, 3.543307};
   public static double starterStackX = 37;
   public final static double starterStackY = 48;

   public final static HashMap<String, Double> fieldColor = new HashMap<String, Double>() {{
      put("Red", 450.0);
      put("Green", 780.0);
      put("Blue", 646.0);
      put("Hue", 0.0);     //STILL NEEDS TO ME MEASURED
   }};
   public final static HashMap<String, Double> launchLineColor = new HashMap<String, Double>() {{
      put("Red", 2490.0);
      put("Green", 4422.0);
      put("Blue", 3775.0);
      put("Hue", 0.0);     //STILL NEEDS TO BE MEASURED
   }};


   public static void setAlliance(Alliances a) {
      if (a != alliance) {
         for (int goal = 0; goal < goalX.length; goal++) {
            goalX[goal] *= -1;
         }
         for (Character zone : targetZoneX.keySet()) {
            targetZoneX.put(zone, -targetZoneX.get(zone));
         }
         starterStackX *= -1;

         alliance = a;
      }
   }
}