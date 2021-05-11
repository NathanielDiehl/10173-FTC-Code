/*
Uses the passed in p, i, and d gains to generate the appropriate corrective value for a hardware device at
any given time. To work properly, these PID controllers will have to be tuned. This code also utilizes a
LinkedList, the values in which can be averaged to generate a more corrective response.
 */

package org.firstinspires.ftc.teamcode.Objects;

import java.lang.Math;
import java.util.LinkedList;

public class PID {
   public double pGain;
   public double iGain;
   public double dGain;
   public double errorMargin;

   private double desiredValue = 0;
   private double currentError = 0;
   private double accumulatedError = 0;
   private double previousError = 0;
   private long previousTime = 0;
   private boolean useD = false;

   private LinkedList<Double> averageList = new LinkedList<>();
   private int averageLength = 1;

   public PID(double p, double i, double d, double e, int a) {
      pGain = p;
      iGain = i;
      dGain = d;
      errorMargin = e;
      averageLength = a;
   }

   public double run(double currentValue, double dv) {
      desiredValue = dv;
      double output = 0;
      double P = 0;
      double I = 0;
      double D = 0;
      long currentTime = System.currentTimeMillis();

      calcCurrentError(currentValue, dv);


      if (shouldRun()) {
         accumulatedError += currentError;

         P = pGain * currentError;
         I = iGain * accumulatedError;
         D = (useD ? 1 : 0) * dGain * ((currentError - previousError) / (currentTime - previousTime));

         previousError = currentError;
         output = (P + I + D);
      }
      previousTime = System.currentTimeMillis();
      useD = true;
      return output;
   }

   public void calcCurrentError(double currentValue, double dv) {
      currentError = currentValue - dv;
   }

   public void notRun() {
      useD = false;
      accumulatedError =0;
      previousError = 0;
      previousTime = System.currentTimeMillis();
   }




   public void reset() {
      accumulatedError = 0;
      previousTime = 0;
   }

   public boolean shouldRun() {
      return (Math.abs(currentError) > errorMargin || Math.abs(desiredValue - getAverage()) > errorMargin);
   }

   public double getCurrentError() {
      return currentError;
   }

   public double getAccumulatedError() {
      return accumulatedError;
   }

   public void addToAverage(double val) {
      averageList.add(val);
      if (averageList.size() > averageLength) {
         averageList.removeFirst();
      }
   }

   public double getAverage() {
      double average = 0;
      for (int i = 0; i < averageList.size(); i++) {
         average += averageList.get(i);
      }
      return (average / averageList.size());
   }
}