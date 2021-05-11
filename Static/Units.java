/*
Converts the passed-in value of the passed-in unit to its equivalent value in the passed-in new unit.
 */


package org.firstinspires.ftc.teamcode.Static;

import java.util.HashMap;

public class Units {

   public enum ValidUnits {INCH, TICK, DEGREE, ROTATION, SECOND, MILLISECOND}

   private final static HashMap<UnitPair, Double> conversionRatios = new HashMap<UnitPair, Double>() {{
      put(new UnitPair(ValidUnits.INCH, ValidUnits.TICK), 252.14444);
      put(new UnitPair(ValidUnits.DEGREE, ValidUnits.TICK), (2678.8 / (2 * Math.PI)) );
      put(new UnitPair(ValidUnits.ROTATION, ValidUnits.TICK), 24.0);
      put(new UnitPair(ValidUnits.SECOND, ValidUnits.MILLISECOND), 1000.0);


      //AUTOMATICALLY INPUTS CONVERSION RATIO RECIPROCALS
      int initLength = this.size();
      for (int i = 0; i < initLength; i++) {
         if (this.size() >= initLength*2) {
            break;
         }

         UnitPair conv = ((UnitPair)(this.keySet().toArray()[i]));
         put(new UnitPair(conv.getConvertTo(), conv.getConvertFrom()), 1 / this.get(conv));
      }
   }};


   public static double convert(double value, ValidUnits from, ValidUnits to) {
         return value * conversionRatios.get(new UnitPair(from, to));
   }
}




class UnitPair {
   private final Units.ValidUnits convertFrom;
   private final Units.ValidUnits convertTo;

   public UnitPair(Units.ValidUnits from, Units.ValidUnits to) {
      convertFrom = from;
      convertTo = to;
   }

   public Units.ValidUnits getConvertFrom() {
      return convertFrom;
   }

   public Units.ValidUnits getConvertTo() {
      return convertTo;
   }

   @Override
   public boolean equals(Object con) {
      if (con instanceof UnitPair) {
         return (convertFrom.equals(((UnitPair) con).getConvertFrom()) && convertTo.equals(((UnitPair) con).getConvertTo()));
      } else {
         return false;
      }
   }

   @Override
   public int hashCode() {
      return (int) Math.pow(convertFrom.hashCode(), convertTo.hashCode());
   }
}