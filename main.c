// Koodi tehty kurssilla Tietokonejärjestelmät (8op)
// Tekijät: Ville Kivikko ja Markus Multamäki


#include <stdio.h>
#include <math.h>
#include <driverlib/aon_batmon.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>

/* Board Header files */
#include "Board.h"
#include "wireless/comm_lib.h"
#include "sensors/bmp280.h"
#include "buzzer.h"
#include "sensors/mpu9250.h"

/* Task */
#define STACKSIZE 2048
Char labTaskStack[STACKSIZE];
Char commTaskStack[STACKSIZE];
Char sensorTaskStack[STACKSIZE];
Char displayTaskStack[STACKSIZE];

/* Display */
Display_Handle hDisplayLcd;

// Pin configuration and variables
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle buzzerHandle;
static PIN_State buzzerState;
static PIN_Handle ledHandle;
static PIN_State ledState;
static PIN_Handle hButtonShut;
static PIN_State bStateShut;

// Global variables
char lampo[16];
char press[16];
char hi5count[16] = "Highfives: 0";
char wavecount[16]= "Handwaves: 0";
char payload[16];
char lahettaja[16];
enum tila { menu=1, eleet, data, viestin_luku };
enum tila myTila = menu;
uint16_t senderAddr = 0x0000;
double pres = 0.00;
double temp = 0.00;
int viesti = 0;
int paivita = 1;

char batteryLevel[16];
int batteryInt;
int batteryFrac;
int REG_BAT_OFFSET = 0x00000028;

void lue_varaustaso();
void soita_musiikkia(int ele);

// Button1 Käynnistys/sammutusnappi
PIN_Config buttonShut[] = {
   Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};
PIN_Config buttonWake[] = {
   Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
   PIN_TERMINATE
};

// Button0
PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};

// Led1
PIN_Config ledConfig[] = {
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, 
   PIN_TERMINATE // Määritys lopetetaan aina tähän vakioon
};

// Buzzer
PIN_Config buzzerConfig[] = {
    Board_BUZZER     | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE 
};


// *******************************
//
// MPU GLOBAL VARIABLES
//
// *******************************
static PIN_Handle hMpuPin;
static PIN_State MpuPinState;
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// *******************************
//
// MPU9250 I2C CONFIG
//
// *******************************
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

void soita_musiikkia(int ele){
    // Tämä funktio soittaa musiikkia tehdyn eleen perusteella.
    // Halutut nuotit annetaan laitteen buzzerille taajuuden perusteella.
    // Funktio myös kannustaa käyttäjää LEDillä.
    
    uint16_t Bb5 = 932;
    uint16_t G6 = 1568;
    uint16_t G5 = 784;
    uint16_t C6 = 1067;
    uint16_t D6 = 1175;
    uint16_t E6 = 1319;
    uint16_t B6 = 1975;
    uint16_t C7 = 2093;
    int i = 0;
    
    uint16_t smokewaterNotes[12] = {G5, Bb5, C6, G5, Bb5, D6, C6, G5, Bb5, C6, Bb5, G5};
    uint16_t smokewaterLengths[12] = {4,4,5,4,4,2,5,4,4,5,4,5};
    uint16_t smokewaterNumberOfNotes = 12;
    
    uint16_t arpeggioNotes[5] = {C6, E6, G6, B6, C7};
    uint16_t arpeggioLengths[5] = {1,1,1,1,1};
    uint16_t arpeggioNumberOfNotes = 5;

    if(ele == 1){
        buzzerOpen(buzzerHandle);
        for (i=0; i<smokewaterNumberOfNotes; i++){
            buzzerSetFrequency(smokewaterNotes[i]);
            PIN_setOutputValue( ledHandle, Board_LED1, !PIN_getOutputValue(Board_LED1) );
            Task_sleep(smokewaterLengths[i]*0.075 * 1000000/Clock_tickPeriod);
            }
        buzzerClose();
    }
    if(ele == 2){
        buzzerOpen(buzzerHandle);
        for (i=0; i<arpeggioNumberOfNotes; i++){
            buzzerSetFrequency(arpeggioNotes[i]);
            PIN_setOutputValue( ledHandle, Board_LED1, !PIN_getOutputValue(Board_LED1) );
            Task_sleep(arpeggioLengths[i] * 0.075 * 1000000/Clock_tickPeriod);
            }
        buzzerClose();
    }
}

Void lue_varaustaso() {
    // Funktio lukee laitteen patterin varaustason ja päivittää sen
    // muuttujaan batteryLevel.
    
    uint32_t battery = HWREG(AON_BATMON_BASE + AON_BATMON_O_BAT);
    batteryInt = (battery >> 8);
    batteryFrac = battery & 0xff;
    Task_sleep(10000 / Clock_tickPeriod);
    sprintf(batteryLevel, "Patteri: %d,%.2d", batteryInt, batteryFrac);
    
}


/* DISPLAY TASK */
Void displayTaskFxn(UArg arg0, UArg arg1) {
    
   // Alustetaan näyttö nyt taskissa
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;

    // Näyttö käyttöön ohjelmassa
    hDisplayLcd = Display_open(Display_Type_LCD, &params);
    tContext *pContext = DisplayExt_getGrlibContext(hDisplayLcd);
  
    
    while (1){
        if (paivita == 1){    
            switch(myTila){
        
            case menu:
            //Tulostetaan alkunäyttö ruudulle
    
            Display_clear(hDisplayLcd);
            Display_print0(hDisplayLcd, 1, 3, "Tervetuloa!");
            Display_print0(hDisplayLcd, 3, 2, "Paina nappia");
            Display_print0(hDisplayLcd, 4, 2, "liikkuaksesi");
            Display_print0(hDisplayLcd, 5, 3, "valikossa.");
    
            if (pContext) {
                // Grafiikkaa näytölle
                GrLineDraw(pContext,0,0,0,96);
                GrLineDraw(pContext,95,0,95,96);
                GrLineDraw(pContext,0,58,95,58);
                // Piirto puskurista näytölle
                GrFlush(pContext);
            }
    
            Display_print0(hDisplayLcd, 9, 1, "Sammuta alem-");
            Display_print0(hDisplayLcd, 10, 1, "masta napista");
            paivita = 0;
            break;
    
            case eleet:
            // Tilassa eleet sensorTaskissa tunnistetaan eleitä, joiden
            // lukumäärä tulostetaan tässä näytölle.
    
            Display_clear(hDisplayLcd);
            Display_print0(hDisplayLcd, 3, 2, "Tunnistetaan");
            Display_print0(hDisplayLcd, 4, 2, "eleita...");
            Display_print0(hDisplayLcd, 6, 2, hi5count);
            Display_print0(hDisplayLcd, 8, 2, wavecount);
            paivita = 0;
            break;
    
            case data:
            // Tulostetaan näytölle lämpötila, paine ja patterin varaustaso.
            // Arvot saadaan globaaleista muuttujista, joita sensorTask päivittää
    
            Task_sleep(100000 / Clock_tickPeriod);
            Display_clear(hDisplayLcd);
            
            if (pContext) {
                // Grafiikkaa näytölle
                GrLineDraw(pContext,0,36,95,36);
                GrLineDraw(pContext,0,68,95,68);
                GrLineDraw(pContext,0,0,95,0);
                GrLineDraw(pContext,0,95,95,95);
                // Piirto puskurista näytölle
                GrFlush(pContext);
            }
            
            Display_print0(hDisplayLcd, 2, 1, lampo);
            Display_print0(hDisplayLcd, 6, 1, press);
            Display_print0(hDisplayLcd, 10, 1, batteryLevel);
            paivita = 0;
            break;
    
            case viestin_luku:
            // Tulostetaan uusi viesti näytölle jos sellainen on.
            
            Display_clear(hDisplayLcd);
            if (viesti == 1){
                Display_print0(hDisplayLcd, 2, 2, "Uusi viesti!");
                Display_print0(hDisplayLcd, 3, 0, payload);
                Display_print0(hDisplayLcd, 5, 1, "Lahettajalta: ");
                Display_print0(hDisplayLcd, 7, 2, lahettaja);
                paivita = 0;
            }
            else{
                Display_print0(hDisplayLcd, 2, 2, "Ei viesteja!");
                paivita = 0;
            }
            break;
            }
        }
        Task_sleep(100000 / Clock_tickPeriod);
    }
}


/* SENSOR TASK */
Void sensorTaskFxn(UArg arg0, UArg arg1) {
    // Tämä taski lukee sensoreilta tietoa, sekä päivittää muuttujien arvot.
    // Samassa taskissa luetaan arvoja joko MPU tai BMP sensorilta, riippuen
    // ohjelman tilasta. Tilassa "eleet" myös tunnistetaan eleitä antureilta
    // saatujen arvojen mukaisesti.
    
    // *******************************
    //
    // USE TWO DIFFERENT I2C INTERFACES
    //
    // *******************************
    
	// INTERFACE FOR OTHER SENSORS
	I2C_Handle i2c;
	I2C_Params i2cParams;
	
	// INTERFACE FOR MPU9250 SENSOR
	I2C_Handle i2cMPU;
	I2C_Params i2cMPUParams;
    float sum = 0;
    int hi5 = 0;
    int handwave = 0;
	float ax, ay, az, gx, gy, gz;
	double pres,temp;

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // *******************************
    //
    // MPU OPEN I2C
    //
    // *******************************
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I222CMPU\n");
    }

    // *******************************
    //
    // MPU POWER ON
    //
    // *******************************
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // WAIT 100MS FOR THE SENSOR TO POWER UP
	Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // *******************************
    //
    // MPU9250 SETUP AND CALIBRATION
    //
    // *******************************
	System_printf("MPU9250: Setup and calibration...\n");
	System_flush();

	mpu9250_setup(&i2cMPU);

	System_printf("MPU9250: Setup and calibration OK\n");
	System_flush();

    // *******************************
    //
    // MPU CLOSE I2C
    //
    // *******************************
    I2C_close(i2cMPU);

    // *******************************
    //
    // BMP280 OPEN I2C
    //
    // *******************************
    i2c = I2C_open(Board_I2C, &i2cParams);
    if (i2c == NULL) {
        System_abort("Error Initializing I2C\n");
    }

    // BMP280 SETUP
    bmp280_setup(&i2c);

    // *******************************
    //
    // BMP280 CLOSE I2C
    //
    // *******************************
    I2C_close(i2c);
    
    // LOOP FOREVER
	while (1) {

        switch(myTila){
            
        case menu:
        // Ei lueta minkään anturin tietoja tässä tilassa.
        break;
        
        case eleet:
        // Tässä tilassa luetaan MPU anturilta kiihtyvyys- ja gyroarvoja, joista
        // kiihtyvyysarvoja käytetään eleiden tunnistamiseen.
        // Myös pariston varaustaso tarkistetaan jo tässä tilassa.
        
        //Päivitä pariston lataustaso
        lue_varaustaso();
	    // *******************************
	    //
	    // MPU OPEN I2C
	    //
	    // *******************************
	    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
	    if (i2cMPU == NULL) {
	        System_abort("Error Initializing I2CMPU\n");
	    }

	    // *******************************
	    //
	    // MPU ASK DATA
		//
        // Accelerometer values: ax,ay,az
	 	// Gyroscope values: gx,gy,gz
		//
	    // *******************************
		mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
        
        sum = sqrt((ax * ax)+(ay * ay)+(az * az));
        
        // Tunnistetaan ylävitonen
        if(sum > 4 && ((ay <(sum/2)) && (ay >-(sum/2)))){
            hi5++;
            Send6LoWPAN(IEEE80154_SERVER_ADDR, "L'A'PSY :D", 10);
            StartReceive6LoWPAN();
            sprintf(hi5count,"Highfives: %d",hi5);
            paivita = 1;
            soita_musiikkia(1);
        }
        
        // Tunnistetaan käden heilutus
        if(sum > 1.7 && (-0.6 < az && az < 0.6)){
            handwave++;
            Send6LoWPAN(IEEE80154_SERVER_ADDR, "moikka :D", 9);
            StartReceive6LoWPAN();
            sprintf(wavecount,"Handwaves: %d",handwave);
            paivita = 1;
            soita_musiikkia(2);
        }
        
	    // *******************************
	    // MPU CLOSE I2C
	    // *******************************
	    I2C_close(i2cMPU);
	    break;
	    
	    case data:
	    // Tässä tilassa luetaan BMP anturilta lämpötila ja ilmanpaine arvot.
	    
	    // *******************************
	    //
	    // BMP280 OPEN I2C
	    //
	    // *******************************
	    
	    i2c = I2C_open(Board_I2C, &i2cParams);
	    if (i2c == NULL) {
	        System_abort("Error Initializing I2C\n");
	    }

	    // *******************************
	    //
	    // BMP280 ASK DATA
	    //
	    // *******************************
	    bmp280_get_data(&i2c, &pres, &temp);
        sprintf(lampo,"Lampo: %.1f\t",temp);
        sprintf(press,"Paine: %.1f\t",pres);
	    // *******************************
	    //
	    // BMP280 CLOSE I2C
	    //
	    // *******************************
	    I2C_close(i2c);
	    break;
        }
	    // WAIT 10MS
    	Task_sleep(100000 / Clock_tickPeriod);
	}
    
}

/* COMMUNICATION TASK */
Void commTaskFxn(UArg arg0, UArg arg1) {
    // Tässä taskissa vastaanotetaan ja luetaan saapuneet viestit.
    
    // Radio to receive mode
	int32_t result = StartReceive6LoWPAN();
	uint8_t maxLen = 0x0f;
	if(result != true) {
		System_abort("Wireless receive mode failed");
	}

    while (1) {

        // If true, we have a message
    	if (GetRXFlag() == true) {
    	    viesti = 1;
    	    memset(payload,0,16);
            Receive6LoWPAN(&senderAddr, &payload, maxLen);
            sprintf(lahettaja,"%x",senderAddr);
    	}

    	// Absolutely NO Task_sleep in this task!!
    }
}

// BUTTON0 KESKEYTYS
void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    // Päivitetään muuttujan myTila arvoa napin painalluksella
    // sekä annetaan näytön päivitysmuuttujalle arvo yksi, jotta näyttö
    // päivittää tilansa.
    
    paivita = 1;
    switch(myTila){
        
    case menu:
    myTila = eleet;
    break;
    
    case eleet:
    myTila = data;
    break;
    
    case data:
    myTila = viestin_luku;
    break;
    
    case viestin_luku:
    if(viesti==1){
        viesti = 0;
    }
    myTila = menu;
    
    break;
    }
}

//Button1 Sammutuspainalluksen käsittelijäfunktio
Void buttonShutFxn(PIN_Handle handle, PIN_Id pinId) {

    // Näyttö pois päältä
    Display_clear(hDisplayLcd);
    Display_close(hDisplayLcd);
   
    //MPU pois päältä
    PIN_setOutputValue(hMpuPin, Board_MPU_POWER, Board_MPU_POWER_OFF);
    PIN_close(ledHandle);
    Task_sleep(100000 / Clock_tickPeriod);
    
    PIN_close(hButtonShut);
    PINCC26XX_setWakeup(buttonWake);
    Power_shutdown(NULL,0);
   
}

Int main(void) {

    // Task variables
	Task_Handle commTask;
	Task_Params commTaskParams;
	Task_Handle sensorTask;
	Task_Params sensorTaskParams;
	Task_Handle displayTask;
	Task_Params displayTaskParams;
	
    // Initialize board
    Board_initGeneral();

    // Buttons and led hande
    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
        System_abort("Error initializing button pins\n");
    }
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
        System_abort("Error initializing LED pins\n");
    };
   
    hButtonShut = PIN_open(&bStateShut, buttonShut);
    if( !hButtonShut ) {
        System_abort("Error initializing button shut pins\n");
    }
    if (PIN_registerIntCb(hButtonShut, &buttonShutFxn) != 0) {
        System_abort("Error registering button callback function");
    }

	// Register the interrupt handler for the button
     if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
      System_abort("Error registering button callback function");
    }

    /* Sensor Task */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTask = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTask == NULL) {
    	System_abort("sensorTask create failed!");
    }
    
    /* Display Task*/
    Task_Params_init(&displayTaskParams);
    displayTaskParams.stackSize = STACKSIZE;
    displayTaskParams.stack = &displayTaskStack;
    displayTaskParams.priority=2;
    displayTask = Task_create(displayTaskFxn, &displayTaskParams, NULL);
    if (displayTask == NULL) {
    	System_abort("displayTask create failed!");
    }

    /* Communication Task */
    Init6LoWPAN(); // This function call before use!

    Task_Params_init(&commTaskParams);
    commTaskParams.stackSize = STACKSIZE;
    commTaskParams.stack = &commTaskStack;
    commTaskParams.priority=1;

    commTask = Task_create(commTaskFxn, &commTaskParams, NULL);
    if (commTask == NULL) {
    	System_abort("Task create failed!");
    }
    System_printf("Hello world!\n");
    System_flush();
    
    /* Start BIOS */
    BIOS_start();

    return (0);
}

