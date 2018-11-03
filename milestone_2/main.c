// Standard includes
#include <string.h>
#include<stdio.h>
#include<stdlib.h>
#include <math.h>
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
#include "gpio.h"
// Common interface includes
#include "uart_if.h"
#include "pinmux.h"
#include "timer.h"
#include "timer_if.h"

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
int adc_opt;
float Volt;
int sampling_rate = 6;
int i;
long buffer [1024];

float frequency ;
int s;
float token;
// FFT declarations
#define q   6       /* for 2^6 points */
#define N   (1<<q)      /* N-point FFT, iFFT */
typedef float real;
typedef struct{real Real_p; real Img_p;} complex;
float count ;


#ifndef PI
# define PI 3.14159265358979323846264338327950288
#endif

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
int f;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//FFt function
void fft( complex *v, int n, complex *tmp )
{
  if(n>1) {         /* otherwise, do nothing and return */
    int k,m;    complex z, w, *vo, *ve;
    ve = tmp; vo = tmp+n/2;
    for(k=0; k<n/2; k++) {
      ve[k] = v[2*k];
      vo[k] = v[2*k+1];
    }
    fft( ve, n/2, v );      /* FFT on even-indexed elements of v[] */
    fft( vo, n/2, v );      /* FFT on odd-indexed elements of v[] */
    for(m=0; m<n/2; m++) {
      w.Real_p = cos(2*PI*m/(double)n);
      w.Img_p = -sin(2*PI*m/(double)n);
      z.Real_p = w.Real_p*vo[m].Real_p - w.Img_p*vo[m].Img_p; /* Re(w*vo[m]) */
      z.Img_p = w.Real_p*vo[m].Img_p + w.Img_p*vo[m].Real_p; /* Im(w*vo[m]) */
      v[  m  ].Real_p = ve[m].Real_p + z.Real_p;
      v[  m  ].Img_p = ve[m].Img_p + z.Img_p;
      v[m+n/2].Real_p = ve[m].Real_p - z.Real_p;
      v[m+n/2].Img_p = ve[m].Img_p - z.Img_p;
    }
  }
  return;
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
void MasterMain()
{
    complex Seq[N], scratch[N];
    int k;
    unsigned long ulUserData;
    unsigned long ulDummy;

    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,MASTER_MSG,sizeof(MASTER_MSG));

    //
    // Set Tx value index
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
    //
    //
    Report("Press any key to start .");

        //
        // Read a character from UART terminal
        //
        ulUserData = MAP_UARTCharGet(UARTA0_BASE);
    while(1)
        {


    //
    // Send the string to slave. Chip Select(CS) needs to be
    // asserted at start of transfer and deasserted at the end.
    //

    for(i=0;i<64;i++)
    {
    MAP_SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
            SPI_CS_ENABLE|SPI_CS_DISABLE);

    // The 10 bit ADC output is read 8 bits at a time
    adc_opt=((int)g_ucRxBuff[0]<<8) | (g_ucRxBuff[1]);
    adc_opt>>=3;
    adc_opt = adc_opt & 0x3FF;


    //convert the adc value to Volt
    Volt = (5.0*adc_opt)/1023;
    fflush(stdin);


    // Input the samples into a variable
    buffer [i] = adc_opt;
    // use lower sampling rate to avoid high frequency noise signals
     MAP_UtilsDelay(13333*sampling_rate);
    }

    for(k=0; k<N; k++)
    {
        Seq[k].Real_p = buffer [k];
        Seq[k].Img_p = 0;
     }
    //Computing FFT

    fft( Seq, N, scratch );


    float max_buffer = (float)Seq[6].Real_p;
    int max_index;
    //limit the frequency form 0.5 Hz to 2 Hz
    for (f = 6; f < 22; f++)
    {
        if (Seq[f].Real_p > max_buffer)
        {
            max_buffer = Seq[f].Real_p;
            max_index = f+1;
        }

    }
    fflush(stdin);
    frequency  = (float)max_index*0.09375;
    printf("frequency :%f",frequency );
    printf("The Detected heart beat:%f",frequency *60);

    if ( frequency *60>=40 && frequency *60 <=60)
    {
      count = 2 ;
    }
    else if ( frequency *60>=61 && frequency *60 <=80)
            {
              count = 4;
            }
        else if ( frequency *60>=61 && frequency *60 <=80)
                    {
                      count = 4 ;
                    }
        else if ( frequency *60>=61 && frequency *60 <=80)
                    {
                      count = 4 ;
                    }
        else if ( frequency *60>=81 && frequency *60 <=100)
                    {
                      count = 6;
                    }
        else if ( frequency *60>= 101 && frequency *60 <=120)
                    {
                      count = 8;
                    }
        else if ( frequency *60>=121 && frequency *60 <=180)
                    {
                      count = 10;
                    }
        token =0;
        while ( token ==0)
        {
            k=0;


               MAP_UtilsDelay(277777/count);
               GPIOPinWrite(GPIOA2_BASE, 0x40,0x40); // output to pin 1 of ULN 2003A (15)
               GPIOPinWrite(GPIOA3_BASE, 0x10,0); // output to pin 2 of ULN 2003A  ( 18)
               GPIOPinWrite(GPIOA0_BASE,0x20,0x20); // output to pin 3 of ULN 2003A  (60)
               GPIOPinWrite(GPIOA0_BASE,0x40,0);  // output to pin 4 of ULN 2003A (61)

               MAP_UtilsDelay(277777/count);
                GPIOPinWrite(GPIOA2_BASE, 0x40,0x40);
                GPIOPinWrite(GPIOA3_BASE, 0x10,0);
                GPIOPinWrite(GPIOA0_BASE,0x20,0);
                GPIOPinWrite(GPIOA0_BASE,0x40,0x40);

                MAP_UtilsDelay(277777/count);
                GPIOPinWrite(GPIOA2_BASE, 0x40,0);
                GPIOPinWrite(GPIOA3_BASE, 0x10,0x10);
                GPIOPinWrite(GPIOA0_BASE,0x20,0);
                GPIOPinWrite(GPIOA0_BASE,0x40,0x40);

                MAP_UtilsDelay(277777/count);
                GPIOPinWrite(GPIOA2_BASE, 0x40,0);
                GPIOPinWrite(GPIOA3_BASE, 0x10,0x10);
                GPIOPinWrite(GPIOA0_BASE,0x20,0x20);
                GPIOPinWrite(GPIOA0_BASE,0x40,0);


        k=k+1;
           if ( k== 120 )
        {
           token = 1;
        }
         }
        }
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
    while(ulUserData != '\r')
    {
        //
        // Read a character from UART terminal
        //
        ulUserData = MAP_UARTCharGet(UARTA0_BASE);

        //
        // Echo it back
        //
        MAP_UARTCharPut(UARTA0_BASE,ulUserData);

        //
        // Push the character over SPI
        //
        MAP_SPIDataPut(GSPI_BASE,ulUserData);

        //
        // Clean up the receive register into a dummy
        // variable
        //
        MAP_SPIDataGet(GSPI_BASE,&ulDummy);
    }

    //
    // Disable chip select
    //
    MAP_SPICSDisable(GSPI_BASE);
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
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()


  {
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

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
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

#if MASTER_MODE

    MasterMain();

#else

    SlaveMain();

#endif

    while(1)
    {

    }

}

