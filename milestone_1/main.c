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
// Application Name     - UART DMA
// Application Overview - The obcountective of this application is to showcase the
//                        use of UART along with uDMA and interrupts. The use
//                        case includes getting input from the user and display
//                        information on the terminal.This example take 8
//                        characters as input which are transfered to a local
//                        buffer using uDMA Rx channel. After receving 8
//                        characters in the local buffer, the caharacters are
//                        send back to the terminal via UART using uDMA Tx channel.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_UART_DMA_Application
// or
// docs\examples\CC32xx_UART_DMA_Application.pdf
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup uart_dma
//! @{
//
//*****************************************************************************
#include<stdio.h>
#include<stdlib.h>
// Driverlib includes
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_uart.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "uart.h"
#include "udma.h"
#include "interrupt.h"
#include "utils.h"
#include "prcm.h"
#include "gpio.h"
// Common interface includes
#include "uart_if.h"
#include "udma_if.h"

#include "pinmux.h"

//*****************************************************************************
//                          MACROS
//*****************************************************************************
#define APPLICATION_VERSION  "1.1.1"
#define APP_NAME            "UART DMA"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile int g_iCounter = 0;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************
static unsigned char ucTextBuff[50];
volatile static tBoolean token;
float count = 1;
//*****************************************************************************
//                      LOCAL DEFINITION
//*****************************************************************************

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
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

static void UARTIntHandler()
{
    //
    // Check if RX
    //
    if(!token)
    {
    //
    // Disable UART RX DMA
    //
        MAP_UARTDMADisable(UARTA0_BASE,UART_DMA_RX);

    //
    // Siganl RX done receiving
    //
        token = true;
    }
    else
    {
    //
    // Disable UART TX DMA
    //
        MAP_UARTDMADisable(UARTA0_BASE,UART_DMA_TX);
    }

    //
    // Clear the UART Interrupt
    //
    MAP_UARTIntClear(UARTA0_BASE,UART_INT_DMATX|UART_INT_DMARX);
}


//*****************************************************************************
//
//! Main function handling the UART and DMA configuration. It takes 8
//! characters from terminal without displaying them. The string of 8
//! caracters will be printed on the terminal as soon as 8th character is
//! typed in.
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void main()
{
    //
    // Initializing the board
    //
    BoardInit();

    //
    // Initialize the RX done flash
    //
    token = false;

    //
    // Initialize uDMA
    //
    UDMAInit();

    //
    // Muxing for Enabling UART_TX and UART_RX.
    //
    PinMuxConfig();

    //
    // Register interrupt handler for UART
    // UARTIntHandler
    MAP_UARTIntRegister(UARTA0_BASE,UARTIntHandler);

    //
    // Enable DMA done interrupts for uart
    //
    MAP_UARTIntEnable(UARTA0_BASE,UART_INT_DMARX);

    //
    // Initialising the Terminal.
    //
    MAP_UARTConfigSetExpClk(CONSOLE,MAP_PRCMPeripheralClockGet(CONSOLE_PERIPH),
                            UART_BAUD_RATE,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE));
    //
    // Clear terminal
    //
    ClearTerm();

    //
    // Display Banner
    //
    Message("\t\t****************************************************\n\r");
    Message("\t\t  Enter the rotations per second ( RPS) from 1-10 \n\r");
    Message("\t\t You can enter a fractional number too between the range!  \n\r");
    Message("\t\t****************************************************\n\r");
    Message("\n\n\n\r");

    //
    // Set the message
    //
    while(1)
    {

    Message("Enter the speed (RPS) you want the motor to run :");

    //
    // Configure the UART Tx and Rx FIFO level to 1/8 i.e 2 characters
    //
    UARTFIFOLevelSet(UARTA0_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);

    //
    // Setup DMA transfer for UART A0
    //
    UDMASetupTransfer(UDMA_CH8_UARTA0_RX,
                      UDMA_MODE_BASIC,
                      8,
                      UDMA_SIZE_8,
                      UDMA_ARB_2,
                      (void *)(UARTA0_BASE+UART_O_DR),
                      UDMA_SRC_INC_NONE,
                      (void *)ucTextBuff,
                      UDMA_DST_INC_8);

    //
    // Enable Rx DMA request from UART
    //
    MAP_UARTDMAEnable(UARTA0_BASE,UART_DMA_RX);

    //
    // Wait for RX to complete
    //
    token = false;
    while(!token)
    {
        if ( count>=1 && count<=10)
        {      // first 7.5 degree Step
                                MAP_UtilsDelay(277777/count);
                                GPIOPinWrite(GPIOA2_BASE, 0x40,0x40); // output to pin 1 of ULN 2003A
                                GPIOPinWrite(GPIOA2_BASE, 0x2,0); // output to pin 2 of ULN 2003A
                                GPIOPinWrite(GPIOA0_BASE,0x20,0x20); // output to pin 3 of ULN 2003A
                                GPIOPinWrite(GPIOA0_BASE,0x40,0);  // output to pin 4 of ULN 2003A

                                   // second Step
                                MAP_UtilsDelay(277777/count);
                                GPIOPinWrite(GPIOA2_BASE, 0x40,0x40);
                                GPIOPinWrite(GPIOA2_BASE, 0x2,0);
                                GPIOPinWrite(GPIOA0_BASE,0x20,0);
                                GPIOPinWrite(GPIOA0_BASE,0x40,0x40);
                                    // Third Step
                                MAP_UtilsDelay(277777/count);
                                GPIOPinWrite(GPIOA2_BASE, 0x40,0);
                                GPIOPinWrite(GPIOA2_BASE, 0x2,0x2);
                                GPIOPinWrite(GPIOA0_BASE,0x20,0);
                                GPIOPinWrite(GPIOA0_BASE,0x40,0x40);
                                    // Fourth Step
                                MAP_UtilsDelay(277777/count);
                                GPIOPinWrite(GPIOA2_BASE, 0x40,0);
                                GPIOPinWrite(GPIOA2_BASE, 0x2,0x2);
                                GPIOPinWrite(GPIOA0_BASE,0x20,0x20);
                                GPIOPinWrite(GPIOA0_BASE,0x40,0);
        }
    }

    Report("%s",ucTextBuff);
    Report("\n \r");

    }
}

