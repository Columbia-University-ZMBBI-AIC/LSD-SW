/**********************************************/
/*  Smart One Shot Led Sync Driver            */
/*  R. Hormigo AIC Columbia University 2021   */
/**********************************************/

#define VERSION "Loading LSD 1.2 with blanking ignore and PMT Gate Shut off"

//Uses STM32Duino core for generic STM32F410CB
//2 needed mods for default UART and Oscillator at 24MHz
//At C:\Users\RICKH\AppData\Local\Arduino15\packages\STM32\hardware\stm32\1.9.0\variants\Generic_F401Cx\variant.h
//#define SERIAL_UART_INSTANCE    1    
//#define PIN_SERIAL_RX           PA10
//#define PIN_SERIAL_TX           PA9
//At C:\Users\RICKH\AppData\Local\Arduino15\packages\STM32\hardware\stm32\1.9.0\system\STM32F4xx\stm32f4xx_hal_conf_default.h
//#define HSE_VALUE              24000000U /*!< Value of the External oscillator in Hz */
//#include <HardwareSerial.h>

//It is prferable to use SWIO/SWCK with ST Link V2 to upload software
//Altertatevily UART1  method at loader port can be used by turning on DIP SW4 and SW6 ON during upload, then turn off

#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SrcWrapper.h>
#include <EEPROM.h>
#include "LSDoled.h"

#define GP_TEST    PB5    //Test Port
#define PMT_GATE   PA11   // PMT Control Gate (Active Low) out pin
#define PMT_GATE_F 0x0800 // PMT Control Gate (Active Low) Fast out pin
#define FB_TRIG    PA15   // Trigger Input Pin
#define FB_TRIG_F  0x8000 // Trigger Input Pin For fast binary read
#define FB_OUT     PB4    // Control Pulse Out Pin
#define FB_OUT_F   0x0010   // Control Pulse Out Pin For fast binary write
#define N_RST      PB8//Reset Bus reading  (old unused D/C in display
#define C1         PA0 //Current 1 
#define C2         PA1 //Current 2 
#define C3         PA2 //Current 3 
#define C4         PA3 //Current 4 
#define C5         PA4 //Current 5 
#define C6         PA5 //Current 6 
#define C7         PA6 //Current 7 
#define C8         PA7 //Current 8 

//Configuration front DIP Switches 

#define AUTO_DIM_SW PC15              //Switch 1 - Auto Dim Function  (ON is Low=AUTO OFF display after few seconds running)
#define AUTO_MAN_SW PC14              //Switch 2 - Auto Man  (ON is Low =AUTO) When Sync is gone, it turns Automatically to Run Free Last M2 (No sync used) with last timing in and asks reset
#define POL_SW PC13                   //Switch 3 - Flyback Polarity (ON is Low = Positive that means FB Out LEDs are ON with FB_TRIG HIGH (Columbia Bruker resonant galvo mode is Negative)
//Used by hardware only (prototype)   //Switch 4 - sets Hardware Boot mode for serial firmware updates. No used with ST Link2
//Used by hardware only (Mod prot)    //Switch 5 - Mask reset from reset bus at LED drivers (ON Masks) To prevent LEDs ON at reset (PMT protection). This needs to be OFF for max current setting.  
#define READ_SW (0x7 & ~(GPIOB->IDR)) //Switches 6-8 in 3 bits, In 2to0 (SEE pDuty[8] for predefined values) duty timing selection: 0 for sync, others for manual modes 3/4


#define GAP_LENGTH 10    //(4) Gap uS before new scan is active (For Auto Mode) 2uS is minimun to overcome circuit latencies
#define DEF_SCAN 500  // Default period us in free runing mode. With no sync present and if Default Period SW (DEF-PER-SW) is on, otherwise Period holds last known value  
#define DIM_DELAY 7000 // 7 + 1(dim) = 8 sec delay to dim display

#define POS true      //Control Output Flyback polarity
#define NEG false

LSDOled oled;
// Pulse Length in uS. Auto Mode is 0, others are Fix in uS, to map with DIP switch selection 
//unsigned int pDutyS[8]={0,1,12,32,64,100,150,255}; 
unsigned int pDutyS[8] = {0,8,16,32,64,125,250,500};
unsigned int pDuty; //Global uS length of the Trigger LEDs Duty pulse (High or Invert Low)
unsigned int pScan; //Global uS length of the Trigger Microscope Scan (Low or Inverted High)
bool resGalvo = false;  //Special resonant Gavlo mode that lights only frame by frame
int currents[8]; //Current setting per each channel (include negatives for tolerance in recover
unsigned int dimDelay;   //Dim control for display 


bool sync = false; //Sync present modes
bool polarity;      //Flyback polarity  POS (true) means FB Out LEDs are ON with FB_TRIG HIGH. (Columbia Bruker is NEG (false) as is reversed, when Laser is on Trigger is high, so LEDs off)
//Basic control state machine, including RUN Modes
typedef enum {SPLASH, TIMING, SET_CURRENT, RUN_SYNC_AUTO, RUN_FREE_LAST, RUN_SYNC_MAN, RUN_FREE_MAN, GO}states;
//HardwareSerial Serial1(PA10, PA9); //Monitoring serial, needed when uploader serial support is off

void updateFromFlash() {
    //Read last parameters from Flash (8 integers address 0 to 7 for currents in mA, 2 more for Pulse Scan and Duty in uS
    //These are the max currents set that can be only read at setting time that internally applies maximum modulation.
    for (byte c = 0; c <= 7; c++) {   //Currents for Channel 1 to 8
        Serial1.print("Last Current ");
        Serial1.print(c + 1);
        Serial1.print(" : ");
        EEPROM.get(c * sizeof(int), currents[c]);
        Serial1.println(currents[c]);
    }
    EEPROM.get(8 * sizeof(int), pDuty); //Last stored duty
    EEPROM.get(9 * sizeof(int), pScan); //Last stored scan
    if ((10 <= pScan && pScan <= 25000) && (5 <= pDuty && pDuty <= 4999)) {  //Validate timing values in memory
        Serial1.print("Last Pulse Duty Length : ");
        Serial1.println(pDuty);
        Serial1.print("Last Pulse Scan Length : ");
        Serial1.println(pScan);
    }
    else {  //Bad values, or corrupted
        pDuty = DEF_SCAN;
        pScan = DEF_SCAN;
        Serial1.print("Bad Duty Length in memory reset to: ");
        Serial1.println(pDuty);
        Serial1.print("Bad Duty Length in memory reset to: ");
        Serial1.println(pScan);
    }
}
void setForm(char* mode) {
    char headL[32], headR[16]; //Display  Headers based in State
    char str_tmp[6];
    if (resGalvo) {
        if (pScan > 999) {
            dtostrf(pScan / 1000.0, 4, 1, str_tmp);
            sprintf(headL, "Res:%smS", str_tmp);
        }
        else
            sprintf(headL, "Res:%04duS", pScan);
        if (pDuty > 999)
            sprintf(headR, "D:%03du", pDuty);
        else
            sprintf(headR, "D:%03duS", pDuty);
    }
    else {
        if (pScan>999)
            sprintf(headL, "Scan:%03du", pScan);
        else
            sprintf(headL, "Scan:%03duS", pScan);
        if (pDuty > 999)
            sprintf(headR, "D:%03du", pDuty);
        else
            sprintf(headR, "D:%03duS", pDuty);
    }
    oled.form(mode, headL, headR, (unsigned int*)currents, dimDelay);
}

void setup() {
    Serial1.begin(9600);
    oled.begin();
    // set the digital pins:
    pinMode(FB_TRIG, INPUT);
    pinMode(FB_OUT, OUTPUT);
    pinMode(PMT_GATE, OUTPUT);
    pinMode(GP_TEST, OUTPUT);
    pinMode(PB0, INPUT_PULLUP); //DIP Front Switch bit 3-0 map to Switch 5-8
    pinMode(PB1, INPUT_PULLUP); //DIP Front Switch bit 3-0 map to Switch 5-8
    pinMode(PB2, INPUT_PULLUP); //DIP Front Switch bit 3-0 map to Switch 5-8
    pinMode(PB3, INPUT_PULLUP); //DIP Front Switch bit 3-0 map to Switch 5-8
    pinMode(AUTO_DIM_SW, INPUT_PULLUP); //Auto Dim display before output driving starts  
    pinMode(AUTO_MAN_SW, INPUT_PULLUP); //Auto Manual if sync goes away
    pinMode(POL_SW, INPUT_PULLUP);      //Polarity of Flyback (On is positive, Duty high)
    pinMode(N_RST, INPUT);
    delay(10); //Settle hardware
    Serial1.println("");
    Serial1.println("==============");
    Serial1.println(VERSION);
    Serial1.print("DIP Switch Timing: ");
    Serial1.print(READ_SW);
    Serial1.print(" - ");
    Serial1.print(pDutyS[READ_SW]);
    Serial1.println("uS");
    Serial1.println("==============");

    dimDelay = (!digitalRead(AUTO_DIM_SW)) ? DIM_DELAY: 0 ;  //Asign either 0 or DIM_DELAY to the Auto Dim Display (0 is no dim) 
    polarity = (digitalRead(POL_SW)) ? NEG : POS ;   //True (DIP switch off) is negative

    /***************************************************
    Main State Machine operation
    SPLASH          Display is shown after reet is pushed down. Wait 3 seconds and move to timing
    TIMING          If reset is hold low, we get in SET_CURRENT mode, otherwise timing is set.
                    In timing we look at the DP 5-8 for values 0-15,
                    0 is auto mode measures and set (same as sync trigger minus GAP_LENGTH),
                    1-15 select duty legth by pDutyS
                    Timeout sync (1 sec)  goes to RUN_FREE
    SET_CURRENT     As reset keeps pushed we set the current values using pots and feedback on display,
                    releasing reset at any time moves back to SPLASH           
    RUN_SYNC_AUTO   M1-Runs sync, with scan/duty times same than sync trigger minus GAP_LENGTH (SW2-ON makes goes to M2 with no Sync, S2-OFF disables out if no sync)
    RUN_FREE_LAST   M2-Runs free driving (no sync) with last known duty/scan timing 
    RUN_SYNC_MAN    M3-Runs sync, same period, but duty time manually defined in DIP SWitch
    RUN_FREE_MAN    M4-Runs free driving (no sync) with period as defined by DEF_PERIOD and duty time manually defined in DIP SWitch
    GO          Execute running as set real time, only a reset can get out of here.              
    *****************************************************/
    states state = (states)SPLASH;
    while(state != (states)GO){
        switch (state){  
            case SPLASH:  
                Serial1.println("SPLASH");
                oled.splash();
                delay(3000);
                updateFromFlash();
                state=(states)TIMING;
                break;
            case TIMING:    
                Serial1.println("TIMING");
                if(digitalRead(N_RST)){ //Reset non asserted (Go timing)
                    state=(states)pulseTiming();
                } 
                else {  //N_RST Asserted (Set Currents)
                    state = (states)SET_CURRENT;
                    Serial1.println("CURRENT SET READ");
                }
                break;
            case SET_CURRENT:
                if(digitalRead(N_RST)){ // Reset released, save and reset operation via Splash
                    Serial1.println("CURRENT SET SAVED");
                    delay(10); //posible debounce noise
                    for (byte c = 0; c <= 7; c++) {   //Currents for Channel 1 to 8
                        int currentStored;
                        EEPROM.get(c * sizeof(int), currentStored);
                        if ((currentStored - 4) > currents[c] || ((currentStored + 4) < currents[c]) || currentStored<0) {
                            EEPROM.put(c * sizeof(int), currents[c]); //Save only values that changed more than +/-4mA (Flash saver)
                            Serial1.print("Current Saved");
                            Serial1.print(c + 1);
                            Serial1.print(" : ");
                            Serial1.println(currents[c]);
                        }
                    }
                    state= (states)SPLASH;
                }
                else{ //Setting
                    for (byte aPort = C1; aPort <= C8; aPort++) {
                        currents[aPort-C1] = analogRead(aPort)*327/100; //Reading all analog currents in array and scaling to current values
                    }
                    oled.form("Setup Current pots...",(unsigned int*)currents);
                    delay(100);  
                }
                break;
            case RUN_SYNC_AUTO: //Mode 1
                Serial1.println("RUN SYNC fully AUTO");
                setForm("SA");
                state = (states)GO;
                break;
            case RUN_FREE_LAST: //Mode 2
                Serial1.println("RUN FREE with LAST Scan / Duty VALUES");
                setForm("FL");
                state = (states)GO;
                break; 
            case RUN_SYNC_MAN:  //Mode 3
                pScan += pDuty - pDutyS[READ_SW];  //pScan (only for display and debug) is override with total original period minus manual pDuty from DIP switch 
                pDuty = pDutyS[READ_SW]; //Override now syn pulse duty length with DIP Switch     
                Serial1.println("RUN SYNC with MANUAL Duty");
                setForm("SM");
                state = (states)GO;
                break;
            case RUN_FREE_MAN:  //Mode 4
                pDuty = pDutyS[READ_SW]; //Set Pulse Duty Length by DIP Switch
                pScan = DEF_SCAN; //Constant hard encoded default scan  
                Serial1.println("RUN FREE with MANUAL Duty and DEF_SCAN ");
                setForm("FM");
                state = (states)GO;
                break;
        }
        delay(100);
    }

    //Flash saver, if value has changed, save Timing in flash before go running
    int pScanStored, pDutyStored;
    EEPROM.get(8 * sizeof(int), pScanStored);
    EEPROM.get(9 * sizeof(int), pDutyStored);
    if (pDutyStored != pDuty ) EEPROM.put(8 * sizeof(int), pDuty);
    if (pScanStored != pScan ) EEPROM.put(9 * sizeof(int), pScan);

    Serial1.print("And go with.. Scan: ");
    Serial1.print(pScan);
    Serial1.print("uS, Duty: ");
    Serial1.print(pDuty);
    Serial1.println("uS");

    delay(100); //Let serial end and turn off
    noInterrupts();  //block serial or other background       
}

//Measuring the trigger Scan and Duty is done by polling here, as IRQs had to much indeterminism and overhead for this application
byte pulseTiming(){
    unsigned long uSIntervalHigh, uSIntervalLow;  //Begining of a trigger and one before in microsecods
    unsigned int dutyLenZ, dutyLenZm1, scanLenZ, scanLenZm1; //Instantaneous values
    unsigned int timeOutCnt = 65535;  //Timeout if no trigger present, this should be under 50mS
    delay(30); // Flush background (like serial via irq) 
    while (!(GPIOA->IDR & FB_TRIG_F) && timeOutCnt) { //Run this while trigger is not falling to low in case is high at calling time... AND timeout still counting down
        timeOutCnt--;
        if (!timeOutCnt){  //If timed Out (timeOutCnt=0) 
            sync = false; //No Sync detected
            if (READ_SW)   //Manual and No Sync, or Sync timed out (Mode 4)
                return RUN_FREE_MAN;
            else  //Auto and No Sync, or Sync timed out (Mode 2)
                return RUN_FREE_LAST;
        }
    }
    //Calculations are made thinking on positive trigger (original design) , for negative Duty and Scan are swapped after calculation
    pDuty = 0; //Reset duty to accumulate new value
    pScan = 0; //Reset scan to accumulate new value
    delayMicroseconds(random(3767)); //Delay the edge ramdomly a few ms to start, so to avoid a blanking timing  gap, to be consistent
    //Duty
    Serial1.print("Deb  "); //Debugging initial timing
    while (GPIOA->IDR & FB_TRIG_F); //Sync it out again, wait for transition to low, so there is time to get in the loop 
    for (unsigned char z=0; z<5; z++){
        while(!(GPIOA->IDR & FB_TRIG_F)); //Wait for rising
        uSIntervalHigh = micros(); //and catch
        while(GPIOA->IDR & FB_TRIG_F); //wait for falling
        dutyLenZm1 = dutyLenZ; //Save last value to vadilate later
        dutyLenZ=micros()-uSIntervalHigh; //and catch the end
        //Validation to ignore gap at vertical Flyback
        pDuty+=dutyLenZ; //Cumulative Values
        if (z && (abs(int(dutyLenZ-dutyLenZm1)>2)) ) { //only when z is > 0, with more than 2uS diference   
            Serial1.print(dutyLenZ); //Debugging duty time
            Serial1.print(":D:");
            Serial1.println(dutyLenZm1);
            return TIMING; //Failed one of the last 5 values, so try again
        }
    }
    //Scan (Low time)
    while (!(GPIOA->IDR & FB_TRIG_F));  //Skip last low to align on next falling, so wait for rising
    for (unsigned char z = 0; z < 5; z++) {
        while (GPIOA->IDR & FB_TRIG_F); //Now Wait for falling...
        uSIntervalLow = micros(); //and catch
        while (!(GPIOA->IDR & FB_TRIG_F)); //wait for rising
        scanLenZm1 = scanLenZ; //Save last value to vadilate later
        scanLenZ = micros() - uSIntervalLow ;  //and catch the end
        //Validation to ignore gap at vertical Flyback
        pScan += scanLenZ; //Cumulative Values
        if (z && (abs(int(scanLenZ - scanLenZm1) > 2))){ //only when z is > 0, with more than 2uS diference       
            Serial1.print(scanLenZ); //Debugging scan time
            Serial1.print(":S:");
            Serial1.println(scanLenZm1);
            return TIMING; //Failed one of the last 5 values, so try again
        }
    }
    
    //Average now 
    if (polarity) { //if Positive 
       pDuty = round((float)pDuty/5.0)-GAP_LENGTH; //Calculated pulse duty average of 5 trigger pulses then short by the gap
       pScan = round((float)pScan/5.0)+GAP_LENGTH; ; //Calculated pulse scan average of 5 trigger pulses then compensate by the gap
    }
    else { //if negative polarity, swapp values
       int pDutyTmp = pDuty;  //Needed to hold swapped value
       pDuty = round((float)pScan/5.0)-GAP_LENGTH; //Calculated pulse duty average of 5 trigger pulses then short by the gap
       pScan = round((float)pDutyTmp/5.0)+GAP_LENGTH; ; //Calculated pulse scan average of 5 trigger pulses then compensate by the gap
    }
    
    sync = true; //Sync detected and measured

    if (pDuty < 32) {
        resGalvo = true;  //Going resonant Galvo Mode that only lights at Horizontal Flyback (No lines)
        resTiming(); //recalculate timing for resonant mode
    }

    if (READ_SW)   //Manual and Active Sync (Mode 3)
        return RUN_SYNC_MAN;
    else  //Auto and Active Sync (Mode 1)
        return RUN_SYNC_AUTO;
}

//Redo Timing after resonant mode (short line scans under 50uS) detected 
void resTiming() {
    unsigned long syncOutCnt, uSIntervalHigh, nowMicros;  //Begining of a trigger and one before in microsecods
    unsigned int dutyLen=0, scanLen=0; //Instantaneous values
    int8_t VBlank = 0, lineLen = 0;   //Counting VBlanks (5 for average) and line length for resonat galvo mode 

    while (VBlank<6) { //Check 6 frames Average the last 4

        while ((GPIOA->IDR & FB_TRIG_F) && lineLen++ < 25) { //  fast digitalRead for trigger. If High is a new scan, start measuring if over 50uS 
            delayMicroseconds(2);
            if (lineLen >= 25) { // this has been over 50uS, must be vertical blanking (flyback)
                nowMicros = micros();
                if (VBlank++>1) scanLen += nowMicros - uSIntervalHigh;//and catch start of duty after ignoring first two
                uSIntervalHigh = nowMicros;
                syncOutCnt = 1000000;  //This is near 80mS in this loop (80 ms here to minimize small duty flash in case FB Trigger IN stays Low) 
                while ((GPIOA->IDR & FB_TRIG_F) && syncOutCnt--); //Hold until end of the vertical flyback "scan" time and add  a bit more
                delayMicroseconds(32);//Extend over flyback "scan"
                if (VBlank > 2) dutyLen += micros() - uSIntervalHigh; //and catch the end
                //Serial1.println(scanLen);
                //Serial1.println(dutyLen);
            }
        }
        lineLen = 0;

    }
    
    pDuty = dutyLen / 4; //Duty time Averaged
    pScan = (scanLen / 4)-pDuty; //Scan time Averaged
    
    Serial1.print("rS:");
    Serial1.println(scanLen);
    Serial1.print("rD:");
    Serial1.println(dutyLen);

}

void loop(){
    int32_t syncOutCnt = 100000; //Watchdog to detect lost of sync
    int8_t lineLen = 0;   //Counting line length for resonat galvo mode 

    //The execution is controlled by polarity first to minimize latencies in sync loop
    //!!POSITIVE POLARITY is not fully tested and DO NOT SUPPORT RESONANT GALVO in this Code Revision until we see a particular need !!
    if (polarity) { // If POSitive, when FB trigger in HIGH, FB LEDs ON (FB trigger HIGH state - gap margin
        while (1) {
            if (sync) {  //Fast Infinite loop with Sync 
                while (syncOutCnt-->0) { 
                        if (GPIOA->IDR & FB_TRIG_F) {//  fast digitalRead for trigger. If High start duty
                            GPIOA->ODR = GPIOA->IDR | PMT_GATE_F; //PMT  gate high -> Shut Down PMT First
                            delayMicroseconds(2);
                            GPIOB->ODR = GPIOB->IDR & ~FB_OUT_F;//Trig Bus Low -> FlyBack/DRIVE LED Out ON
                            delayMicroseconds(pDuty);
                            GPIOB->ODR = GPIOB->IDR | FB_OUT_F; //Trig Bus High -> FlyBack/DRIVE LED Out OFF
                            delayMicroseconds(2);
                            GPIOA->ODR = GPIOA->IDR & ~PMT_GATE_F; //PMT Gate Low -> Restore PMT
                            syncOutCnt = 1000000;  //This is near 80mS in this loop (80 ms here to minimize small duty flash in case FB Trigger IN stays Low) 
                            while (!(GPIOA->IDR & FB_TRIG_F) && syncOutCnt--);  //Wait for original trigger FB duty end (go high) as generated duty is smaller, if no end sync timeout
                        }
                }
                if (!digitalRead(AUTO_MAN_SW)) {   //AUTO MAN is ON  so jump out of auto to no sync
                    interrupts();
                    oled.form("SYNC OFF! Manual Mode", (unsigned int*)currents);
                    delay(3000);
                    oled.form("  Reset when ready   ", (unsigned int*)currents);
                    sync = false;
                    noInterrupts();
                }
                else {
                   syncOutCnt = 100000;  // Sync was lost, try in 10mS go back to Sync Mode. 
                }
            }
            else {  //Fast infinite loop with no sync
                while (1) {
                    GPIOA->ODR = GPIOA->IDR | PMT_GATE_F; //PMT  gate high -> Shut Down PMT First
                    delayMicroseconds(2);
                    GPIOB->ODR = GPIOB->IDR & ~FB_OUT_F; //Turn L only trigger DRIVE Out ON
                    delayMicroseconds(pDuty - 4);
                    GPIOB->ODR = GPIOB->IDR | FB_OUT_F; //Turn H only trigger DRIVE Out OFF
                    delayMicroseconds(2);
                    GPIOA->ODR = GPIOA->IDR & ~PMT_GATE_F; //PMT Gate Low -> Restore PMT
                    delayMicroseconds(pScan);
                }
            }
        }
    }
    else { // If NEGative, with FB trigger in LOW, FB LEDs ON (FB trigger LOW state - gap margin)   --This is Behnia's Bruker 2P Micro
        while (1) {  //For Timeout NEGative mode expects FB triggen IN High in standby (no scanning running) 
            if (sync) {  //Fast Infinite loop with Sync 
                while (syncOutCnt-- > 0) {  //syncOut here can reach to 0 or -1 to be false
                    if (resGalvo) {
                        while ((GPIOA->IDR & FB_TRIG_F) && lineLen++ < 25) { //  fast digitalRead for trigger. If High is a new scan, start measuring if over 50uS 
                            delayMicroseconds(2);
                            if (lineLen >= 25) { // this has been over 50uS, must be vertical blanking (flyback)
                                GPIOA->ODR = GPIOA->IDR | PMT_GATE_F; //PMT  gate high -> Shut Down PMT First
                                delayMicroseconds(2);
                                GPIOB->ODR = GPIOB->IDR & ~FB_OUT_F;//Trig Bus Low -> FlyBack/DRIVE LED Out ON
                                syncOutCnt = 1000000;  //This is near 80mS in this loop (80 ms here to minimize small duty flash in case FB Trigger IN stays Low) 
                                while ((GPIOA->IDR & FB_TRIG_F) && syncOutCnt--); //Hold until end of the vertical flyback "scan" time and add  a bit more
                                delayMicroseconds(32);//Extend over flyback "scan"
                                GPIOB->ODR = GPIOB->IDR | FB_OUT_F; //Trig Bus High -> FlyBack/DRIVE LED Out OFF
                                delayMicroseconds(4);
                                GPIOA->ODR = GPIOA->IDR & ~PMT_GATE_F; //PMT Gate Low -> Restore PMT
                            }
                        }
                        lineLen = 0;
                    }
                    else {
                        if (!(GPIOA->IDR & FB_TRIG_F)) {//  fast digitalRead for trigger. If Low start duty
                            GPIOA->ODR = GPIOA->IDR | PMT_GATE_F; //PMT  gate high -> Shut Down PMT First
                            delayMicroseconds(2);
                            GPIOB->ODR = GPIOB->IDR & ~FB_OUT_F;//Trig Bus Low -> FlyBack/DRIVE LED Out ON
                            delayMicroseconds(pDuty);
                            GPIOB->ODR = GPIOB->IDR | FB_OUT_F; //Trig Bus High -> FlyBack/DRIVE LED Out OFF
                            delayMicroseconds(2);
                            GPIOA->ODR = GPIOA->IDR & ~PMT_GATE_F; //PMT Gate Low -> Restore PMT
                            syncOutCnt = 1000000;  //This is near 80mS in this loop (80 ms here to minimize small duty flash in case FB Trigger IN stays Low) 
                            while (!(GPIOA->IDR & FB_TRIG_F) && syncOutCnt--);  //Wait for original trigger FB duty end (go high) as generated duty is smaller, if no end sync timeout
                        }
                    }
                }

                if (!digitalRead(AUTO_MAN_SW)) {   //AUTO MAN is ON  so jump out of auto to no sync
                    interrupts();
                    oled.form("SYNC OFF! Manual Mode", (unsigned int*)currents);
                    delay(3000);
                    oled.form("  Reset when ready   ", (unsigned int*)currents);
                    sync = false;
                    noInterrupts();
                }
                else if (GPIOA->IDR & FB_TRIG_F){ //Trigger may be back reactivate sync 
                    syncOutCnt = 100000;  // let it some time to start
                } 
            }
            else {  //Fast infinite loop with no sync
                while (1) {
                    GPIOA->ODR = GPIOA->IDR | PMT_GATE_F; //PMT  gate high -> Shut Down PMT First
                    delayMicroseconds(2);
                    GPIOB->ODR = GPIOB->IDR & ~FB_OUT_F; //Turn L only trigger DRIVE Out ON
                    delayMicroseconds(pDuty-4);
                    GPIOB->ODR = GPIOB->IDR | FB_OUT_F; //Turn H only trigger DRIVE Out OFF
                    delayMicroseconds(2);
                    GPIOA->ODR = GPIOA->IDR & ~PMT_GATE_F; //PMT Gate Low -> Restore PMT
                    delayMicroseconds(pScan);
                }
            }

        }
    } 

    
}
