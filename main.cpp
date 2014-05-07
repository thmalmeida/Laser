#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdlib.h>

#include <Arduino.h>
#include "LiquidCrystal.h"
#include "PID_v1.h"

// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)        SCK
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)        MISO
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM    MOSI
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM    SS
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+
//

/*
    atmega168PA  TQFP-32 Connections

    Outputs:

        - PB1 (D9)  OC1A Pin13  - laser PWM
        - PB2 (D10) OC1B Pin14  - Peltier PWM
        - PD2 (D2)       Pin32  - Red Led
        - PD3 (D3)       Pin1   - Green Led

    Inputs:

        - PC5 (D)               - StandBy push button
        - PC4 ()                - Down Push Button
        - PC3 ()    Pin26       - UP push button
        - ADC7      Pin22       - ADC_PD
        - PC0 (A0)  Pin23       - ADC_Laser
        - PC1 (A1)  Pin24       - ADC_Thermistor
        - PC2 (A2)  Pin25       - ADC_ Peltier

    Liquid Crystal:

        - RS
        - E
        - Data4
        - Data5
        - Data6
        - Data7

*/

//RS-E-D4-D5-D6-D7
LiquidCrystal lcd(8, 7, 6, 5, 21, 20);

// Machine State
typedef enum{
    STANDBY,
    INITIALIZING,
    WORKING,
    STOP

}states;
states state = STOP;

float Il, Ip, P, T;

//#define key 1
#define R 6.9//3.64 //71.1
#define Vref 5
#define Iwish 100.0
#define delta 2
#define n_amostras 10
#define Is 150

#define PushButton_DOWN     (~(PINC) & (1<<3))
#define PushButton_UP       (~(PINC) & (1<<4))
#define PushButton_StandBy  (~(PINC) & (1<<5))

uint8_t flag_01 = 0;

void pwm1_init() {

    // --------------------- TIMER1: OUT1 and OUT2 -----------------------
    // Set as output
    DDRB |= (1<<PB1); // (D9)  OUT2 (PB5/OC1A)
    DDRB |= (1<<PB2); // (D10) OUT1 (PB2/OC1B)

    // Go down to avoid peak current on the driver.
    PORTB &= ~(1<<PB1);
    PORTB &= ~(1<<PB2);


    // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR1.
    // CS11: prescale by 8 => 0.5us tick

    // TCCR1A = COM1A1 COM1A0 COM1B1 COM1B0 - - WGM11 WGM10
    TCCR1A = 0b10100010;
    // TCCR1B = ICNC1 ICES1 - WGM13 WGM12 CS12 CS11 CS10
    TCCR1B = 0b00011001;

//    ICR1 = 200; // 40 kHz PWM
	ICR1 = 2500; // 3.2 kHz PWM

    OCR1A = 0; // Init OCR registers to nil output signal
    OCR1B = 0;
    // ------------------------------- END ---------------------------------

    // --------------------- TIMER0: OUT1 and OUT2 -----------------------
//  pinMode(6,1);   // OC0A PD6 OUT1
//  pinMode(5,1);   // OC0B PD5 OUT2

    /*
     * 1 0 1
     * PWM, Phase
     * Correct
     * OCRA TOP BOTTOM
     *
     * OCR2A = 156;
     * f = 16M/((2)*1024*OCR2A) = 50 Hz
     *
     *
     */
    //  TCCR0A ==>> COM0A1 COM0A0 COM0B1 COM0B0 Ã¢â‚¬â€œ Ã¢â‚¬â€œ WGM01 WGM00
//  TCCR0A = 0b00100001;

    // TCCR0B ==>> FOC0A FOC0B Ã¢â‚¬â€œ Ã¢â‚¬â€œ WGM02 CS02 CS01 CS00
//  TCCR0B = 0b00001101;
//  OCR0A = 157;
//  OCR0B = 7;
    // ------------------------------- END ---------------------------------
}
void pwm1_stop()
{
    // --------------------- TIMER1: OUT1 and OUT2 -----------------------
    OCR1A = 0x00; // Init OCR registers to nil output signal
    OCR1B = 0x00;

    // TCCR1A = COM1A1 COM1A0 COM1B1 COM1B0 - - WGM11 WGM10
    TCCR1A = 0b00000010;

    // TCCR1B = ICNC1 ICES1 - WGM13 WGM12 CS12 CS11 CS10
    TCCR1B = 0b00011000;

    // Go down to avoid peak current on the driver.
    PORTB &= ~(1<<PB1);
    PORTB &= ~(1<<PB2);
    // ------------------------------- END ---------------------------------
}
void pwm2_init()
{
    // --------------------- TIMER2: OUT1 and OUT2 -----------------------
    // Is that for the Red Led that indicates StandBy mode while flashing.
    //  pinMode(11,1);  // OC2A OUT1
//      pinMode(3,1);   // OC2B OUT2

    /*
     * 1 0 1
     * PWM, Phase
     * Correct
     * OCRA TOP BOTTOM
     *
     * OCR2A = 156;
     * f = 16M/((2)*1024*OCR2A) = 50 Hz
     *
     *
     */
    //  TCCR2A ==>> COM2A1 COM2A0 COM2B1 COM2B0 - - WGM21 WGM20
    TCCR2A = 0b00000011;

    // TCCR2B ==> FOC2A FOC2B - - WGM22 CS22 CS21 CS20
    TCCR2B = 0b00001010;
    OCR2A = 200;

    // TIMSK2 ==>> - - - - - OCIE2B OCIE2A TOIE2
    TIMSK2 = 0b00000010;

    // Turn Green Led off.
    PORTD &= ~(1<<PD3);

    // TIFR2 ==>> Ã¢â‚¬â€œ Ã¢â‚¬â€œ Ã¢â‚¬â€œ Ã¢â‚¬â€œ Ã¢â‚¬â€œ OCF2B OCF2A TOV2
//  TIFR2 = 0b00000000;

    // ------------------------------- END ---------------------------------

}
void pwm2_stop()
{
    //  TCCR2A ==>> COM2A1 COM2A0 COM2B1 COM2B0 - - WGM21 WGM20
    TCCR2A = 0b00000011;

    // TCCR2B ==> FOC2A FOC2B - - WGM22 CS22 CS21 CS20
    TCCR2B = 0b00001000;
    OCR2A = 200;

    // TIMSK2 ==>> - - - - - OCIE2B OCIE2A TOIE2
    TIMSK2 = 0b00000010;

    // Turn Red Led off.
    PORTD &= ~(1<<PD2);
}
int8_t read_keys(void) // Debounce to push buttons
{
    if(PushButton_UP)
    {
        _delay_ms(20);
        while(PushButton_UP);      // Wait for key to be released
        _delay_ms(20);              // Nao pode comentar, se nao, nao da STOP! 20110816
        return 1;
    }
    else if(PushButton_DOWN)
    {
        _delay_ms(20);
        while(PushButton_DOWN);
        _delay_ms(20);
        return -1;
    }
    else if(PushButton_StandBy)
    {
        _delay_ms(20);
        while(PushButton_StandBy);
        _delay_ms(20);
        return 0;
    }
    else return 2;
}
float I_laser()
{
    double x, I;
    x = (analogRead(0)*Vref)/1023.0;
    I = 1000.0*(x/R);

    return I;
}
float I_peltier()
{
    double x, I;
    x = (analogRead(2)*Vref)/1023.0;
    I = 1000.0*(x/R);

    return I;
}
float PD()
{
    double P;
    P = analogRead(7);

    return P;
}
float Thermistor()
{
    double T;
    T = analogRead(1);

    return T;
}
void safetyCheck()
{
    if(I_laser() >= 100.0)
        state = STOP;

    if(I_peltier() >= 100.0)
        state = STOP;
}
void summary()
{

    // Laser Current
    lcd.setCursor(3,0);
    lcd.print("I ");
    Il = I_laser();
    lcd.print(Il);
    lcd.print("  ");
//
    // Peltier Current
    lcd.setCursor(10,0);
    lcd.print("Ip ");
    Ip = I_peltier();
    lcd.print((int) Ip);
//  lcd.print(OCR1A);
    lcd.print("  ");

    // Photodiode Power
    lcd.setCursor(3,1);
    lcd.print("P ");
    P = PD();
    lcd.print((int) P);
    lcd.print("   ");

    // Temperature thought Thermistor
    lcd.setCursor(10,1);
    lcd.print("T ");
    T = Thermistor();
    lcd.print((int) T);
    lcd.print("   ");

    delay(100);

}

uint16_t a=0;

ISR(TIMER2_COMPB_vect)
{

}
ISR(TIMER2_COMPA_vect)
{
    // 2500 to divide by 5000.
    if(a==2500) {
        a = 0;
        PORTD ^= (1<<PD2);}
    else
        a++;
}



// TEC PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Define the aggressive and conservative Tuning Parameters
//double aggKp=4, aggKi=0.2, aggKd=1;
//double consKp=1, consKi=0.05, consKd=0.25;
double consKp=30, consKi=10, consKd=0;
//Specify the links and initial tuning parameters
PID ThermistorPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);


// Laser PID
double Laser_Setpoint, Laser_Input, Laser_Output;
double Laser_Kp=30, Laser_Ki=10, Laser_Kd=0;
PID LaserPID(&Laser_Input, &Laser_Output, &Laser_Setpoint, Laser_Kp, Laser_Ki, Laser_Kd, DIRECT);


void VerifyPluggedLaser()
{
    if(Thermistor() > 1000)
    {
        if(!flag_01)
        {
            lcd.setCursor(13,0);
            lcd.print("404");
            lcd.setCursor(0,1);
            lcd.print("Laser Not Found");
            flag_01 = 1;
        }
    }
    else
    {
        if(flag_01)
        {
            lcd.setCursor(13,0);
            lcd.print("   ");
            lcd.setCursor(0,1);
            lcd.print("               ");
            flag_01 = 0;
        }
    }
}

#ifndef key
int main() {

    init();

    // Initialize Liquid Crystal
    lcd.begin(16, 2);
    // Input for ADC and pushbuttons
    DDRC &= 0x00;

    int8_t command=3;

    // Set Red and Green leds to output.
    DDRD |= (1<<PD3) | (1<<PD2);

    //initialize the variables we're linked to
    Input = Thermistor();
    Setpoint = 760;
    ThermistorPID.SetMode(AUTOMATIC); //turn the PID on

    //turn the Laser PID on
    Laser_Input = Thermistor();
    Laser_Setpoint = 0;
    LaserPID.SetMode(AUTOMATIC); //turn the PID on

    while(1) {

//      VerifySafetyProtection();
        VerifyPluggedLaser();

        switch (state)
        {
            case STANDBY:

                command = read_keys();
                switch (command)
                {
                    case 0:

                        if(!flag_01)
                        {
                            state = INITIALIZING;
                            lcd.clear();
                            lcd.setCursor(0,0);
                            lcd.print("INITIALIZING");
                        }
                        else
                        {
                            state = STOP;
                            flag_01 = 0;
                        }


                        break;

                    default:
                        break;
                }

                break;

            case INITIALIZING:

                delay(1000);
                pwm1_init();
                pwm2_stop();

                // Set drivers to 0%.
//              OCR1A = 400;
//              OCR1B = 300;

                state = WORKING;
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("O|");
                lcd.setCursor(0,1);
                lcd.print("N|");
                break;

            case WORKING:


                // Working process
                PORTD ^= (1<<PD3);

//              Il = I_laser();
//              if(!((Il < Iwish + delta/2.0) && (Il > Iwish -delta/2.0)))
//              {
//                  if(Il < Iwish)
//                      OCR1A = OCR1A + 1;
//                  else
//                      OCR1A = OCR1A - 1;
//              }
//              else
//              {
//                  summary();
//              }

                // Thermistor Temperature controller through Peltier
                Input = Thermistor();
                ThermistorPID.Compute();
                //analogWrite(3,Output);
                OCR1B = Output;

//                PB2 (D10) OC1B Pin14  - Peltier PWM

                summary();

                command = read_keys();
                switch (command)
                {
                    case 0:
                        state = STOP;
                        lcd.clear();
                        lcd.setCursor(0,0);
                        lcd.print("STOP");
                        break;

                    case 1:
                        OCR1A = OCR1A + 5;

                        break;

                    case -1:
                    	if(OCR1A != 0)
                            OCR1A = OCR1A - 5;

                        break;

                    default:
                        break;
                }

                break;

            case STOP:

                // Stop all pwm outputs
                pwm1_stop();
                delay(1000);
                pwm2_init();

                state = STANDBY;
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("STANDBY");
                lcd.setCursor(12,1);
                lcd.print("v1.0");

                break;


            default:
            break;
        }
    }
}
#else
int main(void) {

    init();

    // Initialize Liquid Crystal
    lcd.begin(16, 2);
    // Input for ADC and pushbuttons
    lcd.print("STANDBY");

    return 0;
}

#endif











//      PORTB |= (1<<PB1);
//      delay(1);
//
//      PORTB &= ~(1<<PB1);
//      delay(40);





//      PORTB |= (1<<PB1);
//      delay(500);
//
//      PORTB &= ~(1<<PB1);
//      delay(500);
//
//
//
//




//
//      switch (state)
//      {
//          case INITIALIZING:
//
//              break;
//
//
//          case WORKING:
//
//              break;
//
//          default:
//          break;
//
//
//
//      }

//      if(!((Ic < Iwish + delta/2) && (Ic > Iwish -delta/2)))
//      {
//          if(Ic < Iwish)
//              OCR1B = OCR1B + 1;
//          else
//              OCR1B = OCR1B - 1;
//
//          x = (analogRead(0)*Vref)/1023;
//          Ic = 1000*(x/R);
//
//          delay(200);
//
//          lcd.setCursor(0,1);
//          lcd.print(Ic);
//
//
//      }


//      PORTD ^= (1<<PD2);
//      PORTD ^= (1<<PD3);
//
//      digitalWrite(9,1);
//
//      lcd.setCursor(0,1);
//      lcd.print(Ic);
//
//      delay(100);
//
//      x = (analogRead(0)*Vref)/1023;
//      Ic = 1000*(x/R);
//
//      if(!((Ic < Iwish + delta/2) && (Ic > Iwish -delta/2)))
//      {
//          if(Ic < Iwish)
//              OCR1B = OCR1B + 1;
//          else
//              OCR1B = OCR1B - 1;
//
////            count = 0;
////            Imin = 1000;
////            Imax = 0;
//
//          x = (analogRead(0)*Vref)/1023;
//          Ic = 1000*(x/R);
//
//          delay(20);
//
//
//      }
//      else
//      {
//          if(count > 100)
//          {
//              Idelta = Imax - Imin;
//              Imin = 1000;
//              Imax = 0;
//              count = 0;
//          }
//          else
//          {
//              if(Ic > Imax)
//              {
//                  Imax = Ic;
//              }
//
//              if(Ic < Imin)
//              {
//                  Imin = Ic;
//              }
//              count++;
//          }
//      }

//void mean()
//{
//  // Calcula a mÃ©dia
////        Im = 0;
////        Is[0] = Is[1];
////        Im = Im +Is[0];
////
////        for(i=1; i<n_amostras; i++)
////        {
////            Is[i] = Is[i+1];
////            Im = Im + Is[i];
////        }
////        Is[n_amostras-1] = Ic;
////        Im = Im/n_amostras;
//}
