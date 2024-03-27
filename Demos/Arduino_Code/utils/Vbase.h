/**
 * @file Vbase.h
 * @author Blake Billharz
 * @brief Vbase is a class to use for calculating left and right wheel voltages from forward and rotational symbolic voltages.
 * @example
 * OOP Use (recommended):
 *      Vbase voltages = Vbase(Vfor, Vrot);
 *      analogWrite(rightWheelPin, voltages.getVright());
 *      analogWrite(leftWheelPin, voltages.getVleft());
 * 
 * Static Implementation Use:
 *      analogWrite(rightWheelPin, Vbase.calcVright(Vfor, Vrot));
 *      analogWrite(leftWheelPin, Vbase.calcVleft(Vfor, Vrot));
 * 
 * @version 0.1
 * @date 2024-02-27
 * 
 */

#ifndef VBASE_H
#define VBASE_H

#include "robotConstants.h"

class Vbase {
    public:

    /**
     * @brief Construct with forward and rotational voltages
     * 
     * @param Vforward Forward symbolic voltage
     * @param Vrot Rotational symbolic voltage
     */
    Vbase(double Vforward, double Vrot) {
        this->Vforward = Vforward;
        this->Vrot = Vrot;

        Vright = (Vforward + Vrot) / 2;
        Vleft = (Vforward - Vrot) / 2;
    }

    /**
     * @brief Default constructor
     * 
     */
    Vbase() {
        Vright = 0;
        Vleft = 0;
        Vforward = 0;
        Vrot = 0;
    }


    // getters
    double getVleft() {
        return Vleft;
    }

    double getVright() {
        return Vright;
    }

    double getVforward() {
        return Vforward;
    }

    double getVrot() {
        return Vrot;
    }


    // setters

    /**
     * @brief Set the forward and rotational values and calculate the left and right voltages.
     * 
     * @param Vforward 
     * @param Vrot 
     */
    void setVoltages(double Vforward, double Vrot) {
        this->Vforward = Vforward;
        this->Vrot = Vrot;

        Vright = (Vforward + Vrot) / 2;
        Vleft = (Vforward - Vrot) / 2;

        // adjust motor voltages for wheel deadzones
        if (Vright > 0) {
            Vright += UNUSABLE_VOLTAGE;
        }
        else if (Vright < 0) {
            Vright -= UNUSABLE_VOLTAGE;
        }
        if (Vleft > 0) {
            Vleft += UNUSABLE_VOLTAGE;
        }
        else if (Vleft < 0) {
            Vleft -= UNUSABLE_VOLTAGE;
        }
    }


    // static method implementation

    /**
     * @brief Static method to calculate Vright from forward and rotational voltages.
     * 
     * @param Vforward Forward symbolic voltage
     * @param Vrot Rotational symbolic voltage
     * @return double - Right wheel voltage
     */
    static double calcVright(double Vforward, double Vrot) {
        return (Vforward + Vrot) / 2;
    }

    /**
     * @brief Static method to calculate Vleft from forward and rotational voltages.
     * 
     * @param Vforward Forward symbolic voltage
     * @param Vrot Rotational symbolic voltage
     * @return double - Left wheel voltage
     */
    static double calcVleft(double Vforward, double Vrot) {
        return (Vforward - Vrot) / 2;
    }


    private:

    double Vleft;
    double Vright;

    double Vforward;
    double Vrot;

};

#endif