

/*******************************************************************************
 * @file        main.c
 * @brief       Source file of the main function of the program
 * @author      Gerardo A. Petroche <gerardo.petroche@gmail.com>
 * @date        2023-02-08
 * @copyright   UVIC
 * @version     v0:[2023-02-08] - Creation file
 * @warning     File not finished
 * @bug         No known bugs
 ******************************************************************************/

#include "ECU_Receiver_main_V1.00.h"
#include <stdio.h>
#include <CircularBuffer.h>
//ECU_Transceiver_Bluetooth




#define HIGH 1
#define LOW 0

#define AdcResolutionValue 0.0054931640625

String lora_band = "865000000";         //Banda de frecuencia (Hz)
String lora_networkid = "18";           //Identificación de la red LORA
String lora_address = "2";              //Dirección del módulo sender
//String lora_RX_address = "1";           //Dirección del módulo receptor receiver 
String lora_mode = "1";         //Transceiver mode
String lora_baudrate = "115200";  //UART Baudrate
String lora_parameter_spreading_factor = "7";  //Cuanto mayor sea el valor de spreading factor, mayor será la distancia de cobertura pero menor será la velocidad de transmisión.
String lora_parameter_bandwidth = "9";     // Un ancho de banda más amplio permite una mayor tasa de bits, pero a costa de una mayor potencia de transmisión.
String lora_parameter_coding_rate = "1";   //Una tasa de codificación más alta proporciona una mayor resistencia a la interferencia, pero a costa de una mayor latencia de transmisión.
String lora_parameter_programmed_preamble = "4"; //Un preámbulo más largo proporciona una mayor robustez en la sincronización, pero a costa de una mayor latencia de transmisión.

String lora_address_receive_data_from = "1";

int BTN_STOP_ALARM = 0;

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

int adc_battery_value;
int adc_fuel_value;
bool NeutralGear_value;
bool OilPressure_value;
bool Key_value;
int Speedometer_value;
int Tachometer_value;

bool telemetry_state = 0;
bool data_not_sent = 0;

double value;

//char  DataIn[40];
int DataIn;
// Variables para parpadeo LED:
uint8_t ledState = 0;
const int LED_ON_TIME_IDLE_MODE = 1000;
const int LED_OFF_TIME_IDLE_MODE = 1000;
unsigned long currentMillis; 
unsigned long previousMillis;

uint32_t data_buffer_tx[7];

void ARDUINO_ISR_ATTR onTimer()     //Se llama a la función cada ms.
{
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

//Buffer circular
CircularBuffer<int,60> bufferRX;       
CircularBuffer<int,30> bufferData;    
int VectorDataRaw[15];
char VectorData[15];
String DataToPc;

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 16, 17);

    

    /*USED GPIOS*/
    pinMode(BatteryADC, INPUT);
    pinMode(Enable_Battery, OUTPUT);
    pinMode(FuelADC, INPUT);
    pinMode(Key, INPUT);
    pinMode(NeutralGear, INPUT);
    pinMode(OilPressure, INPUT);
    pinMode(LORA_NRST, OUTPUT);
    pinMode(Speedometer, INPUT);
    pinMode(Tachometer, INPUT);
    pinMode(LED_Status, OUTPUT);

    digitalWrite(LORA_NRST, HIGH);
    /*NOT USED GPIOS*/
    //pinMode(NOTUSED_2, OUTPUT);
    //pinMode(NOTUSED_4, OUTPUT);
    //pinMode(NOTUSED_5, OUTPUT);
    ////pinMode(NOTUSED_6, OUTPUT);       DO NOT DEFINE AS AN OUTPUT TO GND BECAUSE IT CAUSES uC failures and a reset due the watchdog.
    ////pinMode(NOTUSED_7, OUTPUT);       DO NOT DEFINE AS AN OUTPUT TO GND BECAUSE IT CAUSES uC failures and a reset due the watchdog.
    ////pinMode(NOTUSED_11, OUTPUT);      DO NOT DEFINE AS AN OUTPUT TO GND BECAUSE IT CAUSES uC failures and a reset due the watchdog.
    //pinMode(NOTUSED_12, OUTPUT);
    //pinMode(NOTUSED_13, OUTPUT);
    //pinMode(NOTUSED_15, OUTPUT);
    //pinMode(NOTUSED_19, OUTPUT);
    //pinMode(NOTUSED_26, OUTPUT);
    //pinMode(NOTUSED_27, OUTPUT);
    //pinMode(NOTUSED_32, OUTPUT);
    //pinMode(NOTUSED_33, OUTPUT);
    ////pinMode(NOTUSED_36, OUTPUT);      GPIO34-39 can only be set as input mode and do not have software-enabled pullup or pulldown functions.
    ////pinMode(NOTUSED_39, OUTPUT);      GPIO34-39 can only be set as input mode and do not have software-enabled pullup or pulldown functions.
    //
    //digitalWrite(NOTUSED_2, LOW);
    //digitalWrite(NOTUSED_4, LOW);
    //digitalWrite(NOTUSED_5, LOW);
    ////digitalWrite(NOTUSED_6, LOW);     DO NOT DEFINE AS AN OUTPUT TO GND BECAUSE IT CAUSES uC failures and a reset due the watchdog.
    ////digitalWrite(NOTUSED_7, LOW);     DO NOT DEFINE AS AN OUTPUT TO GND BECAUSE IT CAUSES uC failures and a reset due the watchdog.
    ////digitalWrite(NOTUSED_11, LOW);    DO NOT DEFINE AS AN OUTPUT TO GND BECAUSE IT CAUSES uC failures and a reset due the watchdog.
    //digitalWrite(NOTUSED_12, LOW);
    //digitalWrite(NOTUSED_13, LOW);
    //digitalWrite(NOTUSED_15, LOW);
    //digitalWrite(NOTUSED_19, LOW);
    //digitalWrite(NOTUSED_26, LOW);
    //digitalWrite(NOTUSED_27, LOW);
    //digitalWrite(NOTUSED_32, LOW);
    //digitalWrite(NOTUSED_33, LOW);
    ////digitalWrite(NOTUSED_36, LOW);    GPIO34-39 can only be set as input mode and do not have software-enabled pullup or pulldown functions.
    ////digitalWrite(NOTUSED_39, LOW);    GPIO34-39 can only be set as input mode and do not have software-enabled pullup or pulldown functions.

    digitalWrite(LORA_NRST, LOW);
    digitalWrite(LED_Status, HIGH);
    Serial2.println("AT+BAND=" + lora_band);
    delay(250);
    digitalWrite(LED_Status, LOW);
    delay(250);
    digitalWrite(LED_Status, HIGH);
    Serial2.println("AT+NETWORKID=" + lora_networkid);
    delay(250);
    digitalWrite(LED_Status, LOW);
    delay(250);
    digitalWrite(LED_Status, HIGH);
    Serial2.println("AT+ADDRESS=" + lora_address);
    delay(250);
    digitalWrite(LED_Status, LOW);
    delay(250);
    digitalWrite(LED_Status, HIGH);
    Serial2.println("AT+MODE=" + lora_mode);
    delay(250);
    digitalWrite(LED_Status, LOW);
    delay(250);
    digitalWrite(LED_Status, HIGH);
    Serial2.println("AT+IPR=" + lora_baudrate);
    delay(250);
    digitalWrite(LED_Status, LOW);
    delay(250);
    digitalWrite(LED_Status, HIGH);
    Serial2.println("AT+PARAMETER=" + lora_parameter_spreading_factor + "," + lora_parameter_bandwidth + "," + lora_parameter_coding_rate + "," + lora_parameter_programmed_preamble);
    delay(250);
    digitalWrite(LED_Status, LOW);
    delay(250);
    digitalWrite(LED_Status, HIGH);
    Serial.println("LORA configured!");
    digitalWrite(LED_Status, LOW);
    digitalWrite(LORA_NRST, HIGH);


    timerSemaphore = xSemaphoreCreateBinary();

    // Use 1st timer of 4 (counted from zero).
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
    // info).
    timer = timerBegin(0, 80, true);

    // Attach onTimer function to our timer.
    timerAttachInterrupt(timer, &onTimer, true);

    // Set alarm to call onTimer function every second (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(timer, 1000, true);      //1000000 = 1s     1000 = 1ms

    // Start an alarm
    timerAlarmEnable(timer);
}

void loop() 
{
    currentMillis = millis();
    
    if ((lastIsrAt % 10) == 0)
    {
        flag_10ms = true;
    }
    else
    {
        flag_10ms = false;
    }

    if ((lastIsrAt % 100) == 0)
    {
        flag_100ms = true;
    }
    else
    {
        flag_100ms = false;
    }

    if ((lastIsrAt % 1000) == 0)
    {
        flag_1s = true;
    }
    else
    {
        flag_1s = false;
    }

    bool data_ok = false;

    
    if (Serial2.available())
    {
        //telemetry_state = 1;
        DataIn = Serial2.read();
        //Serial.println(DataIn);
        bufferRX.push(DataIn);
		using index_t = decltype(bufferRX)::index_t;
        data_not_sent = 0;

		for (index_t i = 0; i < bufferRX.size(); i++) 
        {
            if(bufferRX[i] == 61)       //aka "="
            {
                VectorDataRaw[0] = bufferRX[i+4];
                VectorDataRaw[1] = bufferRX[i+5];
                VectorDataRaw[2] = bufferRX[i+6];
                VectorDataRaw[3] = bufferRX[i+7];
                VectorDataRaw[4] = bufferRX[i+8];
                VectorDataRaw[5] = bufferRX[i+9];
                VectorDataRaw[6] = bufferRX[i+10];
                VectorDataRaw[7] = bufferRX[i+11];
                VectorDataRaw[8] = bufferRX[i+12];
                VectorDataRaw[9] = bufferRX[i+13];
                VectorDataRaw[10] = bufferRX[i+14];
                VectorDataRaw[11] = bufferRX[i+15];
                VectorDataRaw[12] = bufferRX[i+16];
                VectorDataRaw[13] = bufferRX[i+17];
                VectorDataRaw[14] = bufferRX[i+18];
                VectorDataRaw[15] = bufferRX[i+19];
            }
            else
            {
                //do nothing
            }
		}
        int counter_delimiter = 0;
        int delimiter = 44;
        int start_index = 0;
        int finish_index = 0;
        int k = 0;

        for (int j=0; j<15 ; j++)
        {
            if(VectorDataRaw[j] == delimiter)
            {
                counter_delimiter = counter_delimiter + 1;
                if (counter_delimiter > 2)
                {
                    data_ok = false;
                    break;
                }
                else
                {
                    //do nothing
                }
                
                for (k = j ; k<15 ; k++)
                {
                    VectorData[k] = VectorDataRaw[k];
                    if(counter_delimiter == 2)
                    {
                        data_ok = true;
                    }
                    else if (counter_delimiter > 2)
                    {
                        data_ok = false;
                        break;
                    }
                    else if (counter_delimiter == 1)    //posiblemente encontrará otra delimiter o no
                    {
                        data_ok = false;
                        //do nothing
                    }
                    else
                    {
                        data_ok = false;
                        break;
                    }
                }
            }
            else
            {
                //do nothing.
            }
        }
        if ((data_ok == true) && (data_not_sent == 0))
        {
            Serial.print(VectorData[0]);  //  Serial.print("  "); delimitador de coma
            Serial.print(VectorData[1]);  //  Serial.print("  "); delimitador de coma
            Serial.print(VectorData[2]);  //  Serial.print("  ");
            Serial.print(VectorData[3]);  //  Serial.print("  ");
            Serial.print(VectorData[4]);  //  Serial.print("  ");
            Serial.print(VectorData[5]);  //  Serial.print("  ");
            Serial.print(VectorData[6]);  //  Serial.print("  ");
            Serial.print(VectorData[7]);  //  Serial.print("  ");
            Serial.print(VectorData[8]);  //  Serial.print("  ");
            Serial.print(VectorData[9]);  //  Serial.print("  ");
            Serial.print(VectorData[10]);  //  Serial.print("  ");
            Serial.print(VectorData[11]);  //  Serial.print("  ");
            Serial.print(VectorData[12]);  //  Serial.print("  ");
            Serial.print(VectorData[13]);  //  Serial.print("  ");
            Serial.print(VectorData[14]);  //  Serial.print("  ");
            Serial.println(VectorData[15]);    //Serial.println("  ");
            data_not_sent = 1;
        }
        else
        {
            data_not_sent = 0;
        }
    
    }
    else
    {
        //misra else
    }
    
    LedManagerProcess(telemetry_state);
    digitalWrite(LORA_NRST, HIGH);
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);
    int counter = isrTime;
}

void LedManagerProcess(bool telemetry_state)
{
    //Serial.println(STATE_MACHINE);
    if (telemetry_state == 1)
    {
        LED_STATUS_WORKING();
    }
    else if (telemetry_state == 0)
    {
        LED_STATUS_IDLE();
    }

    else
    {
        //do nothing
    }
}

/*Parpadeo de led durante NO retransmisión de datos.*/
void LED_STATUS_IDLE(void)
{
    //*Logica de intermitencia de LED en idle function*/
    if (currentMillis - previousMillis >= (ledState == HIGH ? LED_ON_TIME_IDLE_MODE : LED_OFF_TIME_IDLE_MODE)) // Si ha pasado suficiente tiempo
    { 
        previousMillis = currentMillis; // Actualiza el tiempo del último cambio de estado del LED
        ledState = !ledState; // Cambia el estado del LED
        digitalWrite(LED_Status, ledState); // Establece el estado del LED
    }
}

/*Parpadeo de led durante retransmisión de datos.*/
void LED_STATUS_WORKING(void)
{
    if (currentMillis - previousMillis >= 100) // Si ha pasado suficiente tiempo
    { 
        ledState = !ledState;
        previousMillis = currentMillis;

        if (currentMillis - previousMillis >= 50)
        {
            ledState = !ledState;
            previousMillis = currentMillis;

            if (currentMillis - previousMillis >= 100) // Si ha pasado suficiente tiempo
            {
                ledState = !ledState;
                previousMillis = currentMillis;

                if (currentMillis - previousMillis >= 50) // Si ha pasado suficiente tiempo
                {
                    ledState = !ledState;
                    previousMillis = currentMillis;
                }
            }
            else
            {
                //do nothing
            }
    
        }
        else
        {
            //do nothing
        }

    }
    else
    {
        //do nothing
    }
    digitalWrite(LED_Status, ledState);
}

