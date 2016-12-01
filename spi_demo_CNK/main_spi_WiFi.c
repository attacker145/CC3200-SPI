//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - SPI Demo modified
// Application Overview - The demo application focuses on showing the required 
//                        initialization sequence to enable the CC3200 SPI 
//                        module in full duplex 4-wire master and slave mode(s).
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_SPI_Demo
// or
// docs\examples\CC32xx_SPI_Demo.pdf
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "pin.h"
#include "gpio.h"

// Common interface includes
#include "uart_if.h"
#include "pinmux.h"


#define APPLICATION_VERSION     "1.1.1"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


#define FAILURE                 -1
//*****************************************************************************
//
//! SPI Slave Interrupt handler
//!
//! This function is invoked when SPI slave has its receive register full or
//! transmit register empty.
//!
//! \return None.
//
//*****************************************************************************
static void SlaveIntHandler()
{
    unsigned long ulRecvData;
    unsigned long ulStatus;

    ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);

    MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    if(ulStatus & SPI_INT_TX_EMPTY)
    {
        MAP_SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
        ucTxBuffNdx++;
    }

    if(ulStatus & SPI_INT_RX_FULL)
    {
        MAP_SPIDataGetNonBlocking(GSPI_BASE,&ulRecvData);
        g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
        Report("%c",ulRecvData);
        ucRxBuffNdx++;
    }
}

//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void MasterMainSF()
{

    unsigned long ulUserData;
    unsigned long ulDummy;
    //char * 	pcCmdBuffer;
    //pcCmdBuffer = "Hello World";
    //
    // Initialize the message
    //
    //memcpy(g_ucTxBuff,MASTER_MSG,sizeof(MASTER_MSG));
    //g_ucTxBuff[0] = 0x9F;								//
    //g_ucTxBuff[0] = 0x12;
    //g_ucTxBuff[1] = 0x13;
    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    /*
     * pinmux_config.c...
     * //
     * // Configure PIN_15 for GPIO Output
     * //
     * MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
     * MAP_GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_OUT);
     * ...
     * main.c
     * ...
     * MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,GPIO_PIN_6); //Set  Pin 15 to 1
     * MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0); //Set Pin 15 to 0...
     *
     */


    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface Mode 0
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
					 SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Master Mode\n\r");

    //
    // User input
    //
    //Report("Press any key to activate SPI and send data");

    //
    // Read a character from UART terminal
    //
    //ulUserData = MAP_UARTCharGet(UARTA0_BASE);				// Trap


    //
    // Send the string to slave. Chip Select(CS) needs to be
    // asserted at start of transfer and deasserted at the end.
    //
    //MAP_SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
    //        SPI_CS_ENABLE|SPI_CS_DISABLE);
    /*
     * long SPITransfer  (unsigned long ulBase, unsigned char * ucDout, unsigned char * ucDin, unsigned long ulCount, unsigned long ulFlags)
     *
     * Send/Receive data buffer over SPI channel
     * Parameters
     * 	ulBase is the base address of SPI module
     * 	ucDout is the pointer to Tx data buffer or 0.
     * 	ucDin is pointer to Rx data buffer or 0
     * 	ulCount is the size of data in bytes.
     * 	ulFlags controlls chip select toggling.
     *
     * This function transfers ulCount bytes of data over SPI channel.
     * Since the API sends a SPI word at a time ulCount should be a multiple of word length set using
     *
     */
    //MAP_SPITransfer(GSPI_BASE, g_ucTxBuff, g_ucRxBuff, 10, SPI_CS_ENABLE|SPI_CS_DISABLE);

    //
    // Push the character over SPI
    //
    //MAP_SPIDataPut(GSPI_BASE, (unsigned long) g_ucTxBuff[0]);
    //Report("\n\rSent: g_ucTxBuff[0]\t\t %x", g_ucTxBuff[0]);

    //
    // Clean up the receive register into a dummy
    // variable. Waits for the word to be received on the specified port.
    //
    //MAP_SPIDataGet(GSPI_BASE,&ulDummy);



    //
    // Report to the user
    //
    //Report("\n\rSent: g_ucTxBuff[0]\t\t %x",g_ucTxBuff[0]);		//static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
    //Report("\n\rSent: g_ucTxBuff[1]\t\t %x",g_ucTxBuff[1]);
    //Report("\n\rSent: g_ucTxBuff[2]\t\t %x",g_ucTxBuff[2]);


    //Report("\n\rSent\t\t %x",g_ucTxBuff);

    //Report("\n\rReceived: g_ucRxBuff[0]\t\t  %x", &ulDummy);		//<----- Received buffer
    //Report("\n\rReceived: g_ucRxBuff[0]\t\t  %x",g_ucRxBuff[0]);		//<----- Received buffer
    //Report("\n\rReceived: g_ucRxBuff[1]\t\t  %x",g_ucRxBuff[1]);		//<----- Received buffer
    //Report("\n\rReceived: g_ucRxBuff[2]\t\t  %x",g_ucRxBuff[2]);		//<----- Received buffer
    //Report("\n\rReceived\t\t  %x",g_ucRxBuff);							//<----- Received buffer



    //SPICSEnable(GSPI_BASE);
    //for(i = 0; i < TR_BUFF_SIZE; i++)
    //{
    //	SPIDataPut(GSPI_BASE,tx_buffer[i]);
    //	SPIDataGet(GSPI_BASE,&rx_buffer[i]);
    //}



    //
    // Print a message
    //
    Report("\n\rType here (Press s to exit) :");

    //
    // Initialize variable
    //
    ulUserData = 0;

    //
    // Enable Chip select
    //
    MAP_SPICSEnable(GSPI_BASE);

    //
    // Loop until user "Enter Key" is
    // pressed
    //
    //i = 0;
    while(ulUserData != 's')
    {

    	MAP_SPITransfer(GSPI_BASE, g_ucTxBuff, g_ucRxBuff, 17, SPI_CS_ENABLE|SPI_CS_DISABLE);
    	//
    	// Report to the user
    	//
    	Report("\n\rSent: g_ucTxBuff[0]\t\t %x",g_ucTxBuff[0]);		//static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
    	Report("\n\rSent: g_ucTxBuff[1]\t\t %x",g_ucTxBuff[1]);
    	Report("\n\rSent: g_ucTxBuff[2]\t\t %x",g_ucTxBuff[2]);
    	Report("\n\rSent: g_ucTxBuff[3]\t\t %x",g_ucTxBuff[3]);
    	Report("\n\rSent: g_ucTxBuff[4]\t\t %x",g_ucTxBuff[4]);


    	//Report("\n\rSent\t\t %x",g_ucTxBuff);

    	//Report("\n\rReceived: g_ucRxBuff[0]\t\t  %x", &ulDummy);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[0]\t\t  %x",g_ucRxBuff[0]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[1]\t\t  %x",g_ucRxBuff[1]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[2]\t\t  %x",g_ucRxBuff[2]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[3]\t\t  %x",g_ucRxBuff[3]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[4]\t\t  %x",g_ucRxBuff[4]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[5]\t\t  %x",g_ucRxBuff[5]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[6]\t\t  %x",g_ucRxBuff[6]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[7]\t\t  %x",g_ucRxBuff[7]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[8]\t\t  %x",g_ucRxBuff[8]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[9]\t\t  %x",g_ucRxBuff[9]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[10]\t\t  %x",g_ucRxBuff[10]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[11]\t\t  %x",g_ucRxBuff[11]);		//<----- Received buffer
    	Report("\n\rReceived: g_ucRxBuff[12]\t\t  %x",g_ucRxBuff[12]);		//<----- Received buffer
    	Message("\n\r");
    	Report("\n\rReceived: g_ucRxBuff[12]\t\t  %x",g_ucRxBuff);		//<----- Received buffer
    	MAP_UtilsDelay(8000000);
    	Message("\n\n\n\r");
    	//Report("\n\rReceived\t\t  %x",g_ucRxBuff);

    	//
        // Read a character from UART terminal
        //
        ulUserData = MAP_UARTCharGet(UARTA0_BASE);

        //pcCmdBuffer [i] = ulUserData;
        //i++;
        //char * pcInpString = strtok(pcCmdBuffer, " \n\r");
        //Report(pcInpString);
        //
        // Echo it back
        //
        //MAP_UARTCharPut(UARTA0_BASE,ulUserData);

        //
        // Push the character over SPI
        //
        //MAP_SPIDataPut(GSPI_BASE,ulUserData);

        //
        // Clean up the receive register into a dummy
        // variable. Waits for the word to be received on the specified port.
        //
        //MAP_SPIDataGet(GSPI_BASE,&ulDummy);
        //Report("\n\rReceived: Received Buffer\t\t  %x", &ulDummy);
    }

    //
    // Disable chip select
    //
    MAP_SPICSDisable(GSPI_BASE);
}

//*****************************************************************************
//
//! SPI Slave mode main loop
//!
//! This function configures SPI modelue as slave and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void SlaveMain()
{
    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,SLAVE_MSG,sizeof(SLAVE_MSG));

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_SLAVE,SPI_SUB_MODE_0,
                     (SPI_HW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Register Interrupt Handler
    //
    MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);

    //
    // Enable Interrupts
    //
    MAP_SPIIntEnable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Slave Mode\n\rReceived : ");
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Main function for spi demo application 	<----------------------------------------------------------------MAIN
//!
//! \param none
//!
//! \return None.							<----------------------------------------------------------------MAIN
//
//*****************************************************************************
void main()
{
	char acCmdStore[512];
	char *pcInpString;
	int iRetVal = FAILURE;
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    //Enable VCC_BRD
    //
    GPIOPinWrite(GPIOA0_BASE,0x80,0x80); 		//Set  Pin 62 to 1. Enable power to the external electronics
    //GPIOPinWrite(GPIOA0_BASE,0x80,0); 		//Set  Pin 62 to 0. Disable power to the external electronics


    GPIOPinWrite(GPIOA3_BASE,0x40,0x40);		//Turn On Green LED connected to Pin 53

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Display the Banner
    //
    Message("\n\n\n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\t\t        CC3200 SPI Demo Application  \n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\n\n\n\r");

    Message("\n\r Enter command - SF for Serial Flash  \n\r");
    //
    // Get the user command line from PC serial interface and load acCmdStore[512] buffer.
    // Get User Command. GetCmd() is a blocking function, the program will wait for a user input.
    //
    iRetVal = GetCmd(acCmdStore, sizeof(acCmdStore)); // Fill acCmdStore[512] buffer.

    // If an error has occured - long command string
    if(iRetVal < 0)
    {
    //
    // Error in parsing the command as length is exceeded.
    //
    	Message("Command length exceeded 512 bytes \n\r");
    }
    else if(iRetVal == 0)
    {
    //
    // No input. Just an enter pressed probably. Display a prompt.
    //
    }
    else
    {
    //
    // Parse the user command and try to process it.
    //
    //iRetVal = ParseNProcessCmd(acCmdStore);	// acCmdStore[512] buffer is being parsed

    	pcInpString = strtok(acCmdStore, " \n\r"); // " \n\r" - delimiters. Grabs the first string before the whitespace
    	//Make a hardware selection based on the string entered
    	if(!strcmp(pcInpString, "SF")){
    		Message("Serial Flash selected\n\r");

    		//
    		//Set  Pin 15 to 1.
    		//
    		GPIOPinWrite(GPIOA2_BASE,0x40,0x40); 		//Set  Pin 15 to 1. Enable SF CS
    		//GPIOPinWrite(GPIOA2_BASE,0x40,0);			//Set  Pin 15 to 0. Enable ADC

    		//
    		// Configure PIN_16 for GPIO Output							JTAG TDI
    		//
    		PinTypeGPIO(PIN_16, PIN_MODE_0, false);
    		GPIODirModeSet(GPIOA2_BASE, 0x80, GPIO_DIR_MODE_OUT);

    		//
    		//Reset  Pin 16 to 0.
    		//
    		GPIOPinWrite(GPIOA2_BASE,0x80,0x00); 		//Reset  Pin 16 to 0. Enable SF CS

    		// Verify A0 == 1
    		if (GPIOPinRead(GPIOA2_BASE, 0x40)){		//If  Pin 15 set to 1				A0 = 1
    			GPIOPinWrite(GPIOA3_BASE,0x40,0x40); 	//Set  Pin 53 to 1. Enable GRN LED
    		    Message("\n\r");
    		    Message("\t A0 is set to 1");
    		}
    		else{
    		    GPIOPinWrite(GPIOA3_BASE,0x40,0); 		//Set  Pin 53 to 1. Disable GRN LED
    		    Message("\n\r");
    		    Message("\t A0 is set to 0, Error");
    		}

    		// Verify A1 == 0
    		if (!GPIOPinRead(GPIOA2_BASE, 0x80)){			//If  Pin 16 is reset to 0. 		A1 = 0
    		        GPIOPinWrite(GPIOA0_BASE,0x20,0x20); 	//Set  Pin 60 to 1. Enable RED LED
    		    Message("\n\r");
    		    Message("\t A1 is set to 0");
    		    Message("\n\r");
    		 }
    		 else{
    		        GPIOPinWrite(GPIOA0_BASE,0x20,0); 		//Reset  Pin 60 to 0. Disable RED LED
    		    Message("\n\r");
    		    Message("\t A1 is set to 1, Error");
    		 }

    		 if ((GPIOPinRead(GPIOA2_BASE, 0x40)) && (!GPIOPinRead(GPIOA2_BASE, 0x80))){
    		    Message("\t Serial Flash is enabled");
    		    Message("\n\r");
    		 }

    		g_ucTxBuff[0] = 0x9F;

    	}
    	else{
    		Message("ADC selected\n\r");

    		//
    		//Set  Pin 15 to 1.
    		//
    		//GPIOPinWrite(GPIOA2_BASE,0x40,0x40); 		//Set  Pin 15 to 1. Enable SF CS
    		GPIOPinWrite(GPIOA2_BASE,0x40,0);			//Set  Pin 15 to 0. Enable ADC

    		//
    		// Configure PIN_16 for GPIO Output							JTAG TDI
    		//
    		PinTypeGPIO(PIN_16, PIN_MODE_0, false);
    		GPIODirModeSet(GPIOA2_BASE, 0x80, GPIO_DIR_MODE_OUT);

    		//
    		//Reset  Pin 16 to 0.
    		//
    		GPIOPinWrite(GPIOA2_BASE,0x80,0x00); 		//Reset  Pin 16 to 0. Enable SF CS


    		// Verify A0 == 0
    		if (!GPIOPinRead(GPIOA2_BASE, 0x40)){		//If  Pin 15 set to 1				A0 = 1
    			GPIOPinWrite(GPIOA3_BASE,0x40,0x40); 	//Set  Pin 53 to 1. Enable GRN LED
    		    Message("\n\r");
    		    Message("\t A0 is set to 0");
    		}
    		else{
    			GPIOPinWrite(GPIOA3_BASE,0x40,0); 		//Set  Pin 53 to 1. Disable GRN LED
    		    Message("\n\r");
    		    Message("\t A0 is set to 0, Error");
    		}

    		// Verify A1 == 0
    		if (!GPIOPinRead(GPIOA2_BASE, 0x80)){		//If  Pin 16 is reset to 0. 		A1 = 0
    			GPIOPinWrite(GPIOA0_BASE,0x20,0x20); 	//Set  Pin 60 to 1. Enable RED LED
    		    Message("\n\r");
    		    Message("\t A1 is set to 0");
    		    Message("\n\r");
    		}
    		else{
    		    GPIOPinWrite(GPIOA0_BASE,0x20,0); 		//Reset  Pin 60 to 0. Disable RED LED
    		    Message("\n\r");
    		    Message("\t A1 is set to 1, Error");
    		}
    		if ((GPIOPinRead(!GPIOA2_BASE, 0x40)) && (!GPIOPinRead(GPIOA2_BASE, 0x80))){
    		    Message("\t ADC is enabled");
    		    Message("\n\r");
    		}

    		GPIOPinWrite(GPIOA0_BASE,0x80,0); 			//Set START Low
    		GPIOPinWrite(GPIOA1_BASE,0x02,0); 			//Set RESET Low
    		MAP_UtilsDelay(8000);
    		GPIOPinWrite(GPIOA1_BASE,0x02,0x02); 		//Set RESET High
    		GPIOPinWrite(GPIOA0_BASE,0x80,0x80); 		//Set START High

    		//GPIOPinWrite(GPIOA0_BASE,0x80,0x80); 		//Set START high

    		//WriteSPI1(0x40); // WREG command
    		//g_ucTxBuff[0] = 0x40;	// WREG command
    		//WriteSPI1(0x03); // 4 registers - 1 = 0x03
    		//g_ucTxBuff[1] = 0x03;	// 4 registers - 1 = 0x03
    		//WriteSPI1(0x01); // MUX0
    		//g_ucTxBuff[2] = 0x01;	// MUX0
    		//WriteSPI1(0x00); // Vbias
    		//g_ucTxBuff[3] = 0x01;	// Vbias
    		g_ucTxBuff[0] = 0x2A;
    		g_ucTxBuff[1] = 0xFF;
    		//g_ucTxBuff[2] = 0x00;
    	}
    	if(iRetVal < 0)
    	{
    		Message("Error in processing command\n\r");
    	}
    }

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);



#if MASTER_MODE
    //unsigned long ulUserData;
    //ulUserData = MAP_UARTCharGet(UARTA0_BASE);
    //while(ulUserData != '\r')
    //{
    	//
    	// Read a character from UART terminal
    	//
    	//ulUserData = MAP_UARTCharGet(UARTA0_BASE);
    //}

    //if (ulUserData == 'A')
    	MasterMainSF();	// Mode 0

#else

    SlaveMain();

#endif
    GPIOPinWrite(GPIOA3_BASE,0x40,0); 		//Set  Pin 53 to 1. Disable GRN LED
    GPIOPinWrite(GPIOA0_BASE,0x20,0); 		//Reset  Pin 60 to 0. Disable RED LED

    while(1)
    {

    }

}

