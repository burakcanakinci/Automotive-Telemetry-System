

/*******************************************************************************
 * @file        main.c
 * @brief       Source file of the main function of the program
 * @author      Gerardo A. Petroche <gerardo.petroche@gmail.com>
 * @date        2023-02-08
 * @copyright   UVIC
 * @version     v1:[2023-02-08] - Creation file
 * @warning     File not finished
 * @bug         No known bugs
 ******************************************************************************/

#include "ECU_Sender_main_V1.00.h""
#include <stdio.h>
#include "esp_timer.h"

#define HIGH 1
#define LOW 0

String lora_band = "865000000"; //Banda de frecuencia (Hz)
String lora_networkid = "18";   //Identificación de la red Lora
String lora_address = "1";      //Dirección del módulo
String lora_RX_address = "2";   //Dirección del módulo receptor
String lora_mode = "1";         //Transceiver mode
String lora_baudrate = "115200";  //UART Baudrate
String lora_parameter_spreading_factor = "7";  //Cuanto mayor sea el valor de spreading factor, mayor será la distancia de cobertura pero menor será la velocidad de transmisión.
String lora_parameter_bandwidth = "9";     // Un ancho de banda más amplio permite una mayor tasa de bits, pero a costa de una mayor potencia de transmisión.
String lora_parameter_coding_rate = "1";   //Una tasa de codificación más alta proporciona una mayor resistencia a la interferencia, pero a costa de una mayor latencia de transmisión.
String lora_parameter_programmed_preamble = "4"; //Un preámbulo más largo proporciona una mayor robustez en la sincronización, pero a costa de una mayor latencia de transmisión.


uint16_t DataToSend = 1420697050; //BAT_VALUE(3)+FUEL_VALUE(3)+NEUTRAL-OIL-KEY(1)+SPEED(3)+RPM(3) =13

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
bool DEBUG_MODE = 1;
int contador = 0;
int contador_fuel = 0;
int contador_binario = 0; 

double value;

// Variables para parpadeo LED:
uint8_t ledState = 0;
const int LED_ON_TIME_IDLE_MODE = 1000;
const int LED_OFF_TIME_IDLE_MODE = 1000;
unsigned long currentMillis; 
unsigned long previousMillis;
unsigned long lastMillis;

int loops = 0;
unsigned int pulses = 0;

uint32_t data_buffer_tx[7];
int32_t LoraBuffTxAll;
uint32_t LoraBuffTx1;
uint32_t LoraBuffTx2;

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

void IRAM_ATTR countPulses() {
  pulses++;
  //Serial.println(pulses);
}


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
    analogSetWidth(12);

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

    // Watcher for the key value.
    attachInterrupt(Key, KeyWatcher, CHANGE);

    //Asignamos tarea de DataSampling al nucleo 1:
    xTaskCreatePinnedToCore(
    TaskDataSampling, // Function to call
    "DataSampling", // Name for this task, mainly for debug
    1024, // Stack size
    NULL, // pvParameters to pass to the function
    1, // Priority 
    NULL, // Task handler to use
    1 //Core where to run
    );

    //Asignamos tarea de DataSending al nucleo 0:
    xTaskCreatePinnedToCore(
    TaskDataSending, // Function to call
    "DataSending", // Name for this task, mainly for debug
    1024, // Stack size
    NULL, // pvParameters to pass to the function
    1, // Priority 
    NULL, // Task handler to use
    0 //Core where to run
    );
    
    //Interrupción para contar pulsos de velocidad
    attachInterrupt(digitalPinToInterrupt(Speedometer), countPulses, RISING);
}

void loop() 
{

    STATE STATE_MACHINE = STM_PRE_STARTUP;

    // Programa en bucle
    while (1)
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


        switch (STATE_MACHINE)
        {
        case STM_PRE_STARTUP:
            if (LAST_STATE != STM_PRE_STARTUP)
            {
                if (DEBUG_MODE == 1)
                {
                    Serial.println("PRE-STARTUP MACHINE STATE\n");
                }
                else
                {
                    //do nothing
                }
                digitalWrite(LED_Status, HIGH);
                delay(1500);
                Serial2.println("AT+BAND=" + lora_band);
                delay(500);
                Serial2.println("AT+ADDRESS=" + lora_address);
                delay(500);
                Serial2.println("AT+NETWORKID=" + lora_networkid);
                delay(500);
                Serial2.println("AT+MODE=" + lora_mode);
                delay(500);
                Serial2.println("AT+IPR=" + lora_baudrate);
                delay(500);
                Serial2.println("AT+PARAMETER=" + lora_parameter_spreading_factor + "," + lora_parameter_bandwidth + "," + lora_parameter_coding_rate + "," + lora_parameter_programmed_preamble);
                delay(1500);
                digitalWrite(LED_Status, LOW);
            }
            else
            {
                //do nothing
            }
            
            LAST_STATE = STM_PRE_STARTUP;
            STATE_MACHINE = STM_STARTUP;
            break;
        
        case STM_STARTUP:
            if (LAST_STATE != STM_STARTUP)
            {
                if (DEBUG_MODE == 1)
                {
                    Serial.println("START-UP MACHINE STATE\n");
                }
                else
                {
                    //do nothing
                }
                
                
                delay(1000);
            }
            else
            {
                //do nothing
            }
            LAST_STATE = STM_STARTUP;

            //Serial.print("Key_value: "); Serial.println(Key_value);
            if (Key_value == 1)
            {
                 STATE_MACHINE = STM_MAIN;
            }
            else
            {
                //MISRA C++ ELSE
            }
           
            break;

        case STM_MAIN:
            if (LAST_STATE != STM_MAIN)
            {
                if (DEBUG_MODE == 1)
                {
                    Serial.println("MAIN MACHINE STATE\n");
                }
                else
                {
                    //do nothing
                }
                
                
                delay(1000);
            }
            else
            {
                //do nothing
            }
            LAST_STATE = STM_MAIN;

            EnableBatteryReading();

            if (Key_value == 0)
            {
                STATE_MACHINE = STM_SLEEP;
            }
            else
            {
                //MISRA ELSE
            }
            break;

        case STM_SLEEP:
            if (LAST_STATE != STM_SLEEP)
            {
                if (DEBUG_MODE == 1)
                {
                    Serial.println("SLEEP MACHINE STATE\n");
                }
                else
                {
                    //do nothing
                }
                
                delay(1000);
            }
            else
            {
                //do nothing
            }
            LAST_STATE = STM_SLEEP;
            DisableBatteryReading();

            if (Key_value == 1)
            {
                STATE_MACHINE = STM_STARTUP;
            }
            else
            {
                //MISRA ELSE
            }
            break;

        default:
            /* code */
            Serial.println("Invalid State Machine. Rebooting the system...");
            //Añadir acciones a resetear.

            STATE_MACHINE = STM_PRE_STARTUP;
            break; 
        } 
    LedManagerProcess(STATE_MACHINE);
    
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);
    int counter = isrTime;

    }
}

void KeyWatcher(void)
{
    Key_value = getKeyStatus();
    
}


void TaskDataSampling(void *pvParameters)
{
    while (1)
    {
        if (getKeyStatus() == 1)
        {
            if (DEBUG_MODE == 1)
            {
                contador++;
                contador_fuel++;
                contador_binario=0;
                
                adc_battery_value = contador+1;
                adc_fuel_value = contador_fuel;
                NeutralGear_value = contador_binario;
                OilPressure_value = !contador_binario;
                //Key_value = 1;
                Speedometer_value = contador+2;
                Tachometer_value = contador+3;

                if (contador == 250)
                {
                    contador = 0;
                }
                else
                {
                    //do nothing
                }

                if (contador_fuel == 100)
                {
                    contador = 0;
                }
                else
                {
                    //do nothing
                }

                if (contador_binario == 1)
                {
                    contador = 0;
                }
                else
                {
                    //do nothing
                }
            }
            else
            {
                    
                adc_battery_value = getBatteryVoltage();
                adc_fuel_value = getFuel();
                NeutralGear_value = getNeutralGearStatus();
                OilPressure_value = getOilPressureStatus();
                Key_value = getKeyStatus();
                Speedometer_value = getSpeed();
                Tachometer_value = getRPM();

            }

                LoraBuffTxAll = (adc_battery_value & 0xFF) + 
                                ((OilPressure_value & 0b0001) << 8) +
                                ((NeutralGear_value & 0b0001) << 9) +
                                ((Tachometer_value & 0xFF) << 10) +  //Se limita la variable hasta 0xFF valores
                                ((Speedometer_value & 0xFF) << 18) + 
                                ((adc_fuel_value & 0x7F) << 26);

                //Serial.print(LoraBuffTxAll,BIN); Serial.print("   "); Serial.println(LoraBuffTxAll);
                //Serial.print(LoraBuffTx1,BIN); Serial.print("   "); Serial.print(LoraBuffTx1); Serial.print("   "); Serial.print("||"); Serial.print("   "); Serial.print(LoraBuffTx2,BIN); Serial.print("   "); Serial.println(LoraBuffTx2);
                //vTaskDelay(100);
        }
        else
        {
            //Do nothing
        }
    }

}

void LedManagerProcess(STATE STATE_MACHINE)
{
    //Serial.println(STATE_MACHINE);
    if (STATE_MACHINE == STM_MAIN)
    {

        LED_STATUS_WORKING();
        telemetry_state = 1;
    }
    else if ((STATE_MACHINE == STM_PRE_STARTUP) or (STATE_MACHINE == STM_STARTUP) or (STATE_MACHINE == STM_SLEEP))
    {
        LED_STATUS_IDLE();
        telemetry_state = 0;
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

void TaskDataSending(void *pvParameter)
{

     while (1)
    {
        if (getKeyStatus() == 1)
        {
            char CHAR_LoraBuffTxAll[64];
            itoa(LoraBuffTxAll, CHAR_LoraBuffTxAll, 10);
            Serial2.println("AT+SEND=" + lora_RX_address + "," + strlen(CHAR_LoraBuffTxAll) + "," + CHAR_LoraBuffTxAll);
            if (DEBUG_MODE == 1)
            {
                Serial.println("AT+SEND=" + lora_RX_address + "," + strlen(CHAR_LoraBuffTxAll) + "," + CHAR_LoraBuffTxAll);
            }
            //vTaskDelay(2);      //Requiere un Delay minimo de 2 ms para poder enviar correctamente los datos.
            vTaskDelay(5);
        }
        else
        {
            //do nothing
        }
        vTaskDelay(1);  //delay añadido a propósito para evitar que salte el watchdog a causa de un while "vacio" cuando no hay llave
    }
}

/*Devuelve la velocidad convertida de Hz a km/h.*/
uint16_t getSpeed(void)
{
    //45 pulsos por rueda
    //rueda de 17 pulgadas -> 0.4318 metros
    detachInterrupt(digitalPinToInterrupt(Speedometer));
    int Speed = 0;
    const float distancePerPulse = 0.03015; //metros
    float distance = 0;   
    const float interval = 0.01;            //10ms
    
    if ((millis() - lastMillis >= 10) && (flag_10ms == true));
    {

        distance = pulses * distancePerPulse;
        Speed =  distance/interval;
        lastMillis = millis();
        //Serial.print("Speed:"); Serial.print(Speed); Serial.print(" - ");
        //Serial.println(pulses);
        
        pulses = 0;
        flag_10ms = false;
        attachInterrupt(digitalPinToInterrupt(Speedometer), countPulses, RISING);
        
    }
    
    return Speed;
}



