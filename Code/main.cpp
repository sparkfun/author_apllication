/*
 *   Name:Rishabh Berlia
 *   Date:02/23/2016
 *   Program Description: Transmission of Temperature using LEUART to the Adafruit Bluefruit LEUART friend module. If the Temperature is
	 above or below a certain set point an alert is sent. Otherwise the user can request the temperature any time by the command "RetTemp!"
 *   Reference : Lecture, AN0013 (DMA), AN0017(LEUART), Reference Manual, Example codes
 *
 */

/*#includes*/
#include "mbed.h"       // mbed libraries; includes sleep()
#include "em_chip.h"    // for chip related libraries
#include "sleepmodes.h" // to use blockSleepMode() and unblockSleepMode()
#include "em_letimer.h"	// for LETIMER functions; LETIMER_CompareSet, etc
#include "em_acmp.h"	// To use ACMP
#include "em_gpio.h"	// For GPIO setup
#include "em_adc.h"		// For ADC
#include "em_dma.h"		// For DMA
#include "em_cmu.h"		// For Clocks
#include "em_leuart.h"	// For LEUART
#include "string.h"		// For Strings

/* #defines*/
#define DEBUG_ON 0		    // Make this value to 0; To enable Serial print and BCP trace (code-correlation)
#define SLEEPONEXIT 0		// Using SLEEPONEXIT; Even Lower Energy program can be achieved (not configured with DMA)

#define PERIOD 4             // Period is the Time Period in seconds
#define ULFRCO_COUNT 0.001	 // Time for ULFRCO CLOCK to count by 1, therefore Frequency = 1/0.001 = 1000Hz

// DMA Setup Variables
#define DMA_CHANNEL_ADC 2					// DMA channel of ADC (Lowest Priority)
#define DMA_CHANNEL_RX 0					// DMA channel of RX  (Highest Priority)
#define DMA_CHANNEL_TX 1					// DMA channel of TX
#define DMA_PRIORITY_ADC false
#define DMA_PRIORITY_TX  false
#define DMA_PRIORITY_RX  false
#define DMA_ARBITRATE_ADC dmaArbitrate1
#define DMA_ARBITRATE_TX  dmaArbitrate1
#define DMA_ARBITRATE_RX  dmaArbitrate1


// ADC Setup Variables
#define ADC_REF adcRef1V25			// Single Conversion Reference
#define ADC_SAMPLES 200				// The required number of samples

//LEUART Setup Variables
#define LEUART0_BAUD 9600						//Baud rate set to 9600
#define LEUART0_Databits leuartDatabits8		//8 Bits of Data
#define LEUART0_Parity leuartNoParity			//No Parity bit
#define LEUART0_Stopbits leuartStopbits1
#define LEUART0_BUFFER 	1023					//Buffer size for LEUART0 (Max Size), also the RX buffer Size. I chose the Max size just in case.
#define TX_bufferSize 50						//I found my transmit string maximim to be 40 charachters, So I chose 50
uint32_t leuartif;								//LEUART_IF flags variable, to store a copy the IF flag and then using it

// Temperature Settings
#define HIGHTEMP 30								// Temperature Sensor Upper Limit = 30degreesC [LED1]
#define LOWTEMP 15								// Temperature Sensor Upper Limit = 15degreesC [LED0]

/*Global Variables*/
DMA_CB_TypeDef cb;								// For Callback function
volatile uint16_t ADC_Buffer[ADC_SAMPLES];		// Variable to store the ADC values for
volatile char RX_Buffer[LEUART0_BUFFER];		// Receive Buffer of LEUART
float temperature = 0;							// Temperature after conversion
bool trfComplete = false;						// Trf complete flag to check the ADC transfers

//Message Strings to Transfer
char HighTemp[]="Temperature ABOVE set max :";
char LowTemp[]="Temperature BELOW set min :";
char errorMsg[]="\r\nERROR: Not valid input!\r\n";
char returnMsg[]="\r\nTemperature is : ";
char commandString[] = "RetTemp!";


#if DEBUG_ON
Serial pc(USBTX, USBRX);    		// To enable pc.printf() statements; un-comment this and use pc.printf("Text\n\r") for printing text on the terminal
#endif
/*Function prototypes*/

void BSP_TraceSwoSetup(void); 		//Function for Code correlation for mbed and Simplicity studio.

void LETIMER0_IRQHandler(void); 	//Interrupt Handler Function.

void LETIMER_setup(void); 			//LETIMER initial setup function.

void GPIO_Init(void);				//GPIO Initialization Function

void ADC_Setup(void);				//ADC Setup Function

void DMA_Setup(void);				//DMA Setup Function for ADC Channel, Rx Channel, Tx Channel

void DMA_Init(void);				//DMA Initialization function

void LEUART_Setup(void);			//LEUART0 Setup

void DMA_CallBack(unsigned int channel, bool primary, void *user);//DMA CallBack Function (DMA IRQ)


// Common Functions
#if DEBUG_ON
void BSP_TraceSwoSetup(void);	// code correlation function
#endif

/**************************************************************************//**

 * @brief Configure trace output for energyAware Profiler

 * @note Enabling trace will add 80uA current for the EFM32_Gxxx_STK.

 * DK's needs to be initialized with SPI-mode:

 * @verbatim BSP_Init(BSP_INIT_DK_SPI); @endverbatim

 * This Code is from the Assignment 1 example on d2l.
 *****************************************************************************/
void BSP_TraceSwoSetup(void)
{
    /* Enable GPIO clock */

    CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

    /* Enable Serial wire output pin */

    GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;

    /* Set correct location */

    /* This location is valid for GG, LG and WG! */

    GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

    /* Enable output on correct pin. */

    /* This pin is valid for GG, LG and WG! */

    GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);

    GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;

    /* Enable debug clock AUXHFRCO */

    CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

    /* Wait until clock is ready */

    while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY)) ;

    /* Enable trace in core debug */

    CoreDebug->DHCSR |= CoreDebug_DHCSR_C_DEBUGEN_Msk;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Enable PC and IRQ sampling output */

    DWT->CTRL = 0x400113FF;

    /* Set TPIU prescaler to 16. */

    TPI->ACPR = 15;

    /* Set protocol to NRZ */

    TPI->SPPR = 2;

    /* Disable continuous formatting */

    TPI->FFCR = 0x100;

    /* Unlock ITM and output data */

    ITM->LAR = 0xC5ACCE55;

    ITM->TCR = 0x10009;

    /* ITM Channel 0 is used for UART output */

    ITM->TER |= (1UL << 0);

}

/*
 * Function name: convertToCelsius()
 * Description: Converts the ADC raw data to corresponding Celsius.
 */
float convertToCelsius(int32_t adcSample){

    float temp;
    /* Factory calibration temperature from device information page. */
    float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);
    float cal_value_0 = (float)((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK) >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
    /* Temperature gradient (from data sheet) */
    float t_grad= -6.27;
    temp = (cal_temp_0 - ((cal_value_0 - adcSample)/ t_grad));
    return temp;
}

/*
 * Function name: GPIO_Init()
 * Description: Initialize the GPIO pins
 */
void GPIO_Init()
{
    /* Enable clock for GPIO module */
    CMU_ClockEnable(cmuClock_GPIO, true);
	// Pull PD15(CTS pin of BLE module) to GRND
    GPIO_PinModeSet(gpioPortD,15,gpioModePushPull,0);

    /* Configure PE2 (LED0) and PE3(LED1) as Output */
/*  GPIO_PinModeSet(gpioPortE,3,gpioModePushPull,0);
    GPIO_PinModeSet(gpioPortE,2,gpioModePushPull,0);
    GPIO_DriveModeSet(gpioPortE,gpioDriveModeLowest);*/

}
/*
 * Function Name: LETIMER_setup()
 * Description: Configures and starts the LETIMER0
 */

void LETIMER_setup(void)
{
    /* Enabling the required clocks */
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO); //Selecting the ULFRCO as the source clock
    CMU_ClockEnable(cmuClock_LETIMER0, true);           //Enable the clock input to LETIMER

    /* Set the compare values for COMP0 */
    LETIMER_CompareSet(LETIMER0, 0,PERIOD/ULFRCO_COUNT);

    /* Configure the LETIMER0 */
    LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;
    //    {
    //        .enable         = true,                   /* Default: Start counting when init completed. */
    //        .debugRun       = false,                  /* Default: Counter shall not keep running during debug halt. */
    //        .rtcComp0Enable = false,                  /* Default: Don't start counting on RTC COMP0 match. */
    //        .rtcComp1Enable = false,                  /* Default: Don't start counting on RTC COMP1 match. */
    //        .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
    //        .bufTop         = false,                  /* Default: Don't load COMP1 into COMP0 when REP0 reaches 0. */
    //        .out0Pol        = 0,                      /* Default: Idle value for output 0. */
    //        .out1Pol        = 0,                      /* Default: Idle value for output 1. */
    //        .ufoa0          = letimerUFOANone,        /* Default : No action on underfow on Output 0*/
    //        .ufoa1          = letimerUFOANone,        /* Default : No action on underfow on Output 1*/
    //        .repMode        = letimerRepeatFree       /* Default: Count until stopped */
    //    };

    letimerInit.comp0Top= true,                   /* Default: Start counting when init completed. */

    /* Initialize LETIMER with the values above */
    LETIMER_Init(LETIMER0, &letimerInit);
}

/*
 * Function Name: ADC_setup()
 * Description: Configures ADC0
 */
void ADC_Setup(void)

{
    CMU_ClockEnable(cmuClock_ADC0, true);

    // Default Declarations
    ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
    ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

    // ADC Parameters
    init.timebase = ADC_TimebaseCalc(0);
    init.prescale = ADC_PrescaleCalc(857000,0);

    // Initializing ADC
    ADC_Init(ADC0, &init);

    // Single Conversion setup
    // Single Conversion with Repeat ON
    singleInit.reference  = ADC_REF;
    singleInit.input      = adcSingleInpTemp;
    singleInit.resolution = adcRes12Bit;
    singleInit.acqTime = adcAcqTime2;
    singleInit.rep = true;

    // ADC single setup on
    ADC_InitSingle(ADC0, &singleInit);
}
/*
 * Function Name: DMA_Init()
 * Description: Initializes DMA
 */

void DMA_Init(void)
{
    //CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_DMA, true);
    DMA_Init_TypeDef        dmaInit;
    /* Initializing the DMA */
    dmaInit.hprot        = 0;
    dmaInit.controlBlock = dmaControlBlock;
    DMA_Init(&dmaInit);
}


/*
 * Function Name: DMA_setup
 * Description: Setup DMA channels for ADC,TX,RX
 */
void DMA_Setup(void)
{

    DMA_CfgChannel_TypeDef  chnlCfg_adc;
    DMA_CfgDescr_TypeDef    descrCfg_adc;
    DMA_CfgChannel_TypeDef  chnlCfg_rx;
    DMA_CfgDescr_TypeDef    descrCfg_rx;
    DMA_CfgChannel_TypeDef  chnlCfg_tx;
    DMA_CfgDescr_TypeDef    descrCfg_tx;

    /* Setting up call-back function */
    cb.cbFunc  = DMA_CallBack;
    cb.userPtr = NULL;

    /* Setting up ADC channel */
    chnlCfg_adc.highPri   = DMA_PRIORITY_ADC;
    chnlCfg_adc.enableInt = true;
    chnlCfg_adc.select    = DMAREQ_ADC0_SINGLE;
    chnlCfg_adc.cb        = &cb;
    DMA_CfgChannel(DMA_CHANNEL_ADC, &chnlCfg_adc);

    /* Setting up ADC channel descriptor */
    descrCfg_adc.dstInc  = dmaDataInc2;
    descrCfg_adc.srcInc  = dmaDataIncNone;
    descrCfg_adc.size    = dmaDataSize2;
    descrCfg_adc.arbRate = DMA_ARBITRATE_ADC;
    descrCfg_adc.hprot   = 0;
    DMA_CfgDescr(DMA_CHANNEL_ADC, true, &descrCfg_adc);

    /* Starting DMA transfer using Basic since every transfer must be initiated by the ADC. */
       DMA_ActivateBasic(DMA_CHANNEL_ADC, true, false, (void *)ADC_Buffer, (void *)&(ADC0->SINGLEDATA), ADC_SAMPLES - 1);

    /* Setting up Rx channel */
    chnlCfg_rx.highPri   = DMA_PRIORITY_RX;
    chnlCfg_rx.enableInt = true;
    chnlCfg_rx.select    = DMAREQ_LEUART0_RXDATAV;
    chnlCfg_rx.cb        = &cb;
    DMA_CfgChannel(DMA_CHANNEL_RX, &chnlCfg_rx);

    /* Setting up Rx channel descriptor */
    descrCfg_rx.dstInc  = dmaDataInc1;
    descrCfg_rx.srcInc  = dmaDataIncNone;
    descrCfg_rx.size    = dmaDataSize1;
    descrCfg_rx.arbRate = DMA_ARBITRATE_RX;
    descrCfg_rx.hprot   = 0;
    DMA_CfgDescr(DMA_CHANNEL_RX, true, &descrCfg_rx);

    /* Setting up Tx channel */
    chnlCfg_tx.highPri   = DMA_PRIORITY_TX;
    chnlCfg_tx.enableInt = true;
    chnlCfg_tx.select    = DMAREQ_LEUART0_TXBL;
    chnlCfg_tx.cb        = &cb;
    DMA_CfgChannel(DMA_CHANNEL_TX, &chnlCfg_tx);

    /* Setting up Tx channel descriptor */
    descrCfg_tx.dstInc  = dmaDataIncNone;
    descrCfg_tx.srcInc  = dmaDataInc1;
    descrCfg_tx.size    = dmaDataSize1;
    descrCfg_tx.arbRate = DMA_ARBITRATE_TX;
    descrCfg_tx.hprot   = 0;
    DMA_CfgDescr(DMA_CHANNEL_TX, true, &descrCfg_tx);

}

/*
 * Function Name: ADC_setup
 * Description: Configures ADC0
 */
void LEUART_Setup(void)
{
	/* Enabling the required clocks */
	CMU_ClockEnable(cmuClock_LFB, true);           //Enable the clock input to LETIMER
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO); //Selecting the ULFRCO as the source clock
	CMU_ClockEnable(cmuClock_LEUART0, true);           //Enable the clock input to LETIMER
	/* Defining the LEUART1 initialization data */
		LEUART_Init_TypeDef leuart0Init =
		{
		  .enable   = leuartEnable,        // Activate data reception on LEUn_TX pin.
		  .refFreq  = 0,                   // Inherit the clock frequency from the LEUART clock source
		  .baudrate = LEUART0_BAUD,    // Baudrate = 9600 bps
		  .databits = LEUART0_Databits,    // Each LEUART frame contains 8 databits
		  .parity   = LEUART0_Parity,      // No parity bits in use
		  .stopbits = LEUART0_Stopbits,    // Setting the number of stop bits in a frame to 2 bitperiods
		};

		LEUART_Init(LEUART0, &leuart0Init);

		// Route LEUART1 TX,RX pin to DMA location 0
		LEUART0->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN | LEUART_ROUTE_LOCATION_LOC0;

		// Enable GPIO for LEUART1. TX is on D4
		GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 0);
		// Enable GPIO for LEUART1. RX is on D5
		GPIO_PinModeSet(gpioPortD, 5, gpioModeInputPull, 0);
		// Pull PD15(CTS pin of BLE module) to GRND
		GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 0);

}

/*
 *Function name: LETIMER0_IRQHandler
 *Description : Interrupt Service Routine for LETIMER.
 */
void LETIMER0_IRQHandler(void)
{
    LETIMER_IntClear(LETIMER0, LETIMER_IF_UF); //Clear LETIMER0 underflow (UF) and COMP1 flag.
    DMA_ActivateBasic(DMA_CHANNEL_ADC, true, false, (void *)ADC_Buffer, (void *)&(ADC0->SINGLEDATA), ADC_SAMPLES - 1);
    ADC_Setup();
    // ADC start
    ADC_Start(ADC0, adcStartSingle);
    unblockSleepMode(EM2);
    blockSleepMode(EM1);
    trfComplete=false;
}

/*
 *Function name: LEUART0_IRQHandler
 *Description : Interrupt Service Routine for LEUART0.
 */
void LEUART0_IRQHandler(void)
{
	leuartif = LEUART_IntGet(LEUART0); 	// Store the interrupt flag

	LEUART_IntClear(LEUART0, leuartif); //Clear interrupts

	if (leuartif & LEUART_IF_SIGF)
	{
		int temp = 0, i = 0;
		char tempChar[7];						// To store the Value of Temperature in char to transmit
		char temp_string[TX_bufferSize];	// Concatenated string for Message and Temperature, this will be transfered
		char copyCmp[sizeof(commandString)/sizeof(char)];							// string to store the Received string from Buffer to compare

		// Stop the LEUART reception and DMA Transfers
		for (i = 0; i < (strlen(commandString)); i++)		// Run loop till the return message (RetTemp!) length and copy RX buffer to copyCmp for comparing

		{
			copyCmp[i] = RX_Buffer[i];
		}

		copyCmp[8]='\0';

    	/* To extract the digits of the temperature variable and put in tempChar. A basic digit extraction algorithm is used and then each digit is passed one by one. */
		if (!strcmp(commandString,copyCmp)) 			// If valid Command is Received ie RetTemp!
		{
			temp = temperature*10;
			tempChar[0] = (temp/100)+48;
			temp = temp%100;
			tempChar[1] = (temp/10)+48;
			temp  = temp%10;
			tempChar[2] = '.';
			tempChar[3] = (temp)+48;
			tempChar[4] = 'C';
			tempChar[5] = '\r';
			tempChar[6] = '\n';

			strcpy(temp_string,returnMsg);					//Copy the returnMsg message in the temporary string
			strcat(temp_string,tempChar);					// Concatenate with tempChar to get the final message to be transfered

			// Enable DMA wake-up from LEUART0 TX
			LEUART0->CTRL = LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
			// Activate DMA for LEUART TX transfers
			DMA_ActivateBasic(DMA_CHANNEL_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)temp_string, strlen(temp_string)- 1);	// -1 for the Null character which we wont transmit
		}
		else
		{
			LEUART0->CTRL = LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART0 TX in EM2
			// Activate DMA for LEUART TX transfers
			DMA_ActivateBasic(DMA_CHANNEL_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)errorMsg, strlen(errorMsg)-2);	// -1 for the Null character which we wont transmit
		}
		LEUART_IntEnable(LEUART0, LEUART_IF_RXDATAV); //Enable RXDATA Interrupt to check for received characters
		DMA_ActivateBasic(DMA_CHANNEL_RX, true, false, NULL, NULL, LEUART0_BUFFER-1);
	}
	else if (leuartif & LEUART_IF_RXDATAV)
	{
		LEUART_IntDisable(LEUART0, LEUART_IF_RXDATAV);	// Disable after receiving
	}

}

/*
 *Function name: DMA_CallBack()
 *Description :  Call back function of the DMA
 */
void DMA_CallBack(unsigned int channel, bool primary, void *user)
{
    unblockSleepMode(EM1);
    blockSleepMode(EM2);

	if(!trfComplete)
	{
		ADC_Reset(ADC0);		// Reset the ADC; Turn it off
	    //ADC0->CMD = ADC_CMD_SINGLESTOP;

		int temp = 0, i = 0;
		    char tempChar[7];			//To store the temperature in char, for transmitting
		    char temp_string[TX_bufferSize];

		    (void) channel;
		    (void) primary;
		    (void) user;

		    for (i = 0; i < ADC_SAMPLES; i++)
		    {
		        temp += (ADC_Buffer[i]);
		    }
		    temperature = temp / ADC_SAMPLES;
		    temperature = convertToCelsius(temperature); //Get value of Temperature in deg C


		    if (temperature > HIGHTEMP)
		    {
		        //GPIO_PinOutClear(gpioPortE,2);
		        //GPIO_PinOutSet(gpioPortE,3);
		    	/* To extract the digits of the temperature variable and put in tempChar. A basic digit extraction algorithm is used and then each digit is passed one by one. */
		        temp 		= temperature*10;
				tempChar[0] = (temp/100)+48;
				temp 	    = temp%100;
				tempChar[1] = (temp/10)+48;
				temp 		= temp%10;
				tempChar[2] = '.';
				tempChar[3] = (temp)+48;
				tempChar[4] = 'C';			// Pad the 4th position of charTemp as C
				tempChar[5] = '\r';			// Pad carriage return
				tempChar[6] = '\n';			// Pad line feed

				strcpy(temp_string,HighTemp);					// Copy the HighTemp message in the temporary string
				strcat(temp_string,tempChar);					// Concatenate with the tempChar to get the final message

    			LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
    			// Activate DMA transfers for LEUART TX
		        DMA_ActivateBasic(DMA_CHANNEL_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)temp_string, strlen(temp_string) - 1);

		    }
		    else if (temperature < LOWTEMP)
		    {
		        //GPIO_PinOutSet(gpioPortE,2);
		        //GPIO_PinOutClear(gpioPortE,3);

		        temp 		= temperature*10;
				tempChar[0] = (temp/100)+48;
				temp 	    = temp%100;
				tempChar[1] = (temp/10)+48;
				temp 		= temp%10;
				tempChar[2] = '.';
				tempChar[3] = (temp)+48;
				tempChar[4] = 'C';
				tempChar[5] = '\r';
				tempChar[6] = '\n';

				strcpy(temp_string,LowTemp);					// Copy the LowTemp message in the temporary string
				strcat(temp_string,tempChar);					// Concatenate with the tempChar to get the final message

				LEUART0->CTRL |= LEUART_CTRL_TXDMAWU;				// Enable DMA wake up for LEUART TX in EM2
				// Activate DMA transfers for LEUART TX
				DMA_ActivateBasic(DMA_CHANNEL_TX, true, false, (void *)&(LEUART0->TXDATA), (void *)temp_string, strlen(temp_string) -1);

		    }
		   trfComplete=true;
	}

	else
	{
		(void) channel;
		(void) primary;
		(void) user;

		// Disable DMA wake-up from LEUART1 TX
		LEUART0->CTRL &= ~LEUART_CTRL_TXDMAWU;
	}
}

/*
 * Function Name: main();
 * Description: All the function calls are done in main(). The CPU goes to sleep while in main(); until Interupt is generated.
 */
int main(void)
{

    CHIP_Init();

    CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
    CMU_OscillatorEnable(cmuOsc_HFXO, false, false);

    blockSleepMode(EM2); //Prevents the CPU to go below EM3 mode.

#if DEBUG_ON
    BSP_TraceSwoSetup(); //For simplicity studio Energy profiler code correlation.
#endif
    LETIMER_setup(); //Initialize LETIMER.

    ADC_Setup(); //Initialize the ADC

    DMA_Init();	//Initialize DMA.

    DMA_Setup(); //Setup DMA.

    LEUART_Setup(); //Initialize LEUART.

    GPIO_Init(); //Initialize GPOIs.

    LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF); //Enable underflow UF interrupt.

    LEUART_IntEnable(LEUART0, LEUART_IF_SIGF);	// Enable SF RXDATAV

    NVIC_EnableIRQ(LETIMER0_IRQn); //Enable LETIMER0 interrupt vector in NVIC (Nested Vector Interrupt Controller)

    NVIC_EnableIRQ(LEUART0_IRQn); //Enable LETIMER0 interrupt vector in NVIC (Nested Vector Interrupt Controller)

	LEUART0->SIGFRAME = '!';							// Set LEUART signal frame to '!'

	LEUART0->CTRL |= LEUART_CTRL_RXDMAWU;				// Enable DMA wake up for LEUART RX in EM2
    DMA_ActivateBasic(DMA_CHANNEL_RX, true, false, (void *)RX_Buffer, (void *)&(LEUART0->RXDATA), LEUART0_BUFFER-1);

    // Enable Sleep-on-Exit
#if SLEEPONEXIT
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;	// Setting the corresponding bit for SleepOnExit
#endif

    while(1)
    {
        sleep(); //CPU goes to EM3 Mode to save energy, waits there until Interrupt is generated.
    }

}
