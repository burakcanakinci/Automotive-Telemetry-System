

/*******************************************************************************
 * @file        main.h
 * @brief       Source file of the main function of the program
 * @author      Gerardo A. Petroche <gerardo.petroche@gmail.com>
 * @date        2023-02-08
 * @copyright   UVIC
 * @version     v1:[2023-02-08] - Creation file
 * @warning     File not finished
 * @bug         No known bugs
 ******************************************************************************/

#ifndef ECU_Sender_main_V1.00_H
#define ECU_Sender_main_v1.00_H

/*GPIOS DEFINE REGION*/
//#define NOTUSED_0 0
//#define UART_0_TX 1       Already define on HardwareSerial.cpp file
#define NOTUSED_2 2
//#define UART0_RX 3        Already define on HardwareSerial.cpp file
#define NOTUSED_4 4
#define NOTUSED_5 5
#define NOTUSED_6 6
#define NOTUSED_7 7
#define NOTUSED_8 8       //PENDIENTE DE CONFIRMAR
//#define UART_1_RX 9       Already define on HardwareSerial.cpp file
//#define UART_1_TX 10      Already define on HardwareSerial.cpp file
#define NOTUSED_11 11
#define NOTUSED_12 12
#define Key 13              //Confirmed
#define Tachometer  14      //Confirmed
#define NOTUSED_15 15
//#define UART_2_RX 16       //UART_RX2 defined on HardwareSerial.cpp file
//#define UART_2_TX 17       //UART_TX2 defined on HardwareSerial.cpp file
#define Enable_Battery  18  //Confirmed
#define NOTUSED_19 19
// GPIO20 NC per specification
#define LORA_NRST 21        //Confirmed
#define LED_Status 22       //Confirmed
#define Speedometer 23      //Confirmed
// GPIO24 NC per specification
#define NeutralGear 25      //Confirmed
#define NOTUSED_26 26
#define OilPressure 27      //Confirmed
//GPIO28 NC per specification
//GPIO29 NC per specification
//GPIO30 NC per specification
//GPIO31 NC per specification
#define NOTUSED_32 32
#define NOTUSED_33 33
#define BatteryADC 34       //Confirmed
#define FuelADC 35          //Confirmed
#define NOTUSED_36 36
//GPIO37 NC per specification
//GPIO38 NC per specification
#define NOTUSED_39 39

/*Global variables*/
int rawValue_BatteryVoltage;
int BatteryVoltage;
bool EN_BAT_ADC;
int rawValue_Fuel;
bool rawValue_Key;
bool rawValue_NeutralGear;
bool rawValue_OilPressure;
uint16_t rawValue_SpeedPulse;
uint16_t rawValue_RpmPulse;
int FuelValue;

#define AdcResolutionValue_VBat  0.00087912
#define CoefAdjust_Vbat 56.68
#define AdcResolutionValue_Fuel  0.0014652
#define CoefAdjust_Fuel 18.08427882

bool flag_1ms = false;
bool flag_10ms = false;
bool flag_100ms = false;
bool flag_1s = false;

typedef enum
{
    STM_PRE_STARTUP,
    STM_STARTUP,
    STM_MAIN,
    STM_SLEEP
} STATE;

int LAST_STATE;


/*Devuelve señal analógica.*/
uint8_t getBatteryVoltage(void)
{
    rawValue_BatteryVoltage = analogRead(BatteryADC);
    BatteryVoltage = (AdcResolutionValue_VBat*rawValue_BatteryVoltage)*CoefAdjust_Vbat;
    //AdcResolution = Vmax/BitsADC -> 3.6/4095=0.00087912
    //Coeficiente para aproximar la tensión a la entrada de la adaptación de niveles (divisor de tensión)
    return BatteryVoltage;
}

/*Señal de salida GPIO EN_BAT_ADC.*/
void EnableBatteryReading(void)
{
    digitalWrite(Enable_Battery, HIGH);
}

void DisableBatteryReading(void)
{
    digitalWrite(Enable_Battery, LOW);
}

/*Obtiene señal analógica de fuel mediante ADC.*/
uint8_t getFuel(void)
{
    rawValue_Fuel = analogRead(FuelADC);
    FuelValue = (AdcResolutionValue_Fuel*rawValue_Fuel)*CoefAdjust_Fuel;
    //AdcResolution = Vmax/BitsADC -> 6/4095=0.0014652
    //Coeficiente para aproximar la tensión a la entrada de la adaptación de niveles (divisor de tensión)
    return FuelValue;
}

/*Obtiene señal digital 1 o 0 del estado de llave.*/
bool getKeyStatus(void)
{
    rawValue_Key = digitalRead(Key);
    return rawValue_Key;
}

/*Obtiene señal digital 1 o 0 del estado de la marcha neutral engranada.*/
bool getNeutralGearStatus(void)
{
    rawValue_NeutralGear = digitalRead(NeutralGear);
    return rawValue_NeutralGear;
}

/*Obtiene señal digital 1 o 0 del estado de la presión de aceite.*/
bool getOilPressureStatus(void)
{
    rawValue_OilPressure = digitalRead(OilPressure);
    return rawValue_OilPressure;
}



/*Devuelve las revoluciones del motor convertidas de Hz a RPM.*/
uint16_t getRPM(void)
{
    int rpm = 0;
    return rpm;
}



#endif

