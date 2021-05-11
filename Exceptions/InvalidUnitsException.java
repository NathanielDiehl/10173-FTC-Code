/*
This is never actually used, however it can be if you make the Units class throw it. This exception would
theoretically be thrown if you asked the Units class to convert between units which cannot be converted between
or if you ask it to use a unit which has not been defined in the ValidUnits enum variable.
 */

package org.firstinspires.ftc.teamcode.Exceptions;

public class InvalidUnitsException extends Exception {
    public InvalidUnitsException(String errorMessage) {
        super(errorMessage);
    }
}


