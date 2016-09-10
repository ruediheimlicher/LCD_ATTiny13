//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
//#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>


#include "adc.c"

//***********************************
//Reset							*
//									*
//***********************************



#define LOOPLEDPIN            0        // Blink-LED
#define STARTPIN                1        // Eingang von Taste fuer Start
#define POTPIN				2			// Eingang vom Pot fuer Helligkeit
#define OSZIPIN               4
#define PWMPIN             3       // Wie OSZI. Meldet Reset an Webserver, active LO



#define RESETCOUNT            0x200   // Fehlercounter: Zeit bis Reset ausgeloest wird
#define RESETDELAY            0x08   // Waitcounter: Blockiert wiedereinschalten
#define WEBSERVERRESETDELAY   0x010
#define WAIT                  0

#define SDA_LO_RESET          2 // gesetzt, wenn SDA zulange LO ist
#define SDA_HI_RESET          3 // gesetzt, wenn SDA zulange HI ist

#define SDA_LO_MAX            0x8000
#define SDA_HI_MAX            0xFFFF

void delay_ms(unsigned int ms);

volatile uint16_t	loopcount0=0;
volatile uint16_t	loopcount1=0;

volatile uint16_t	delaycount=0; // Zaehlt wenn WAIT gesetzt ist: Delay fuer Relais

volatile uint16_t	webserverresetcount=0; // Zeit, die der Resetrequest vom Webserver dauert

volatile uint8_t statusflag=0;

volatile uint16_t	overflowcount=0;

volatile uint16_t	ANZEIGE_LO_counter=0;
volatile uint16_t	ANZEIGE_HI_counter=0;


void slaveinit(void)
{
   /*
    #define LOOPLEDPIN            0        // Blink-LED
    #define STARTPIN               1        // Eingang von Taste fuer Start
    #define POTPIN				2			// Eingang vom Pot fuer Helligkeit
    #define OSZIPIN               4
    #define PWMPIN             3       // Wie OSZI. Meldet Reset an Webserver, active LO
    */
    
    CLKPR |= (1<<3);
   
    DDRB |= (1<<LOOPLEDPIN);
    PORTB |= (1<<LOOPLEDPIN);
   
    DDRB |= (1<<PWMPIN);       // Ausgang: MOSFET PWM
    PORTB &= ~(1<<PWMPIN);     // LO
    
    DDRB |= (1<<OSZIPIN);        // Ausgang
    PORTB |= (1<<OSZIPIN);       // HI
    
    DDRB &= ~(1<<STARTPIN);        // Eingang: Verbunden Taste fuer Start Anzeige
    PORTB |= (1<<STARTPIN);        // HI
    

   DDRB &= ~(1<<POTPIN);        // Eingang: Verbunden mit Pot fuer Helligkeit
   PORTB |= (1<<POTPIN);        // HI

   
   
   
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms)
	{
		_delay_ms(0.96);
		ms--;
	}
}

/* Initializes the hardware timer  */
void timer_init(void)
{
	/* Set timer to CTC mode */
	TCCR0A = (1 << WGM00)|(1 << WGM01);
	/* Set prescaler */
	TCCR0B = (1 << CS00)|(1 << CS01); // clock/64
	/* Set output compare register for 1ms ticks */
	//OCR0A = (F_CPU / 8) / 1000;
	/* Enable output compare A interrupts */
	TIMSK0 = (1 << TOIE0)|(1<<OCIE0A); // TOV0 Overflow, compA
   OCR0A = 0x0F;
}

ISR(TIM0_OVF_vect) //
{
   if (statusflag & (1<<WAIT))
   {
   PORTB |= (1<<PWMPIN);
   }
   if (statusflag & (1<<WAIT)) // Display ist noch am Laufen
   {
      ANZEIGE_LO_counter++;
      if (ANZEIGE_LO_counter>0x2FFF)
      {
         statusflag &= ~(1<<WAIT);
       //  PORTB |= (1<<OSZIPIN);
      }
   }
   
}

ISR(TIM0_COMPA_vect) //
{
   //statusflag |= (1<<CHECK);
   PORTB &= ~(1<<PWMPIN);
}



ISR(INT0_vect) // Potential-Aenderung von INT0, Taster. Setzt Zeitfenster
{
   
   
   if ((!(statusflag & (1<<WAIT))))// WAIT verhindert, dass LCD nicht sofort wieder zurueckgesetzt wird
   {
      //
      PORTB &= ~(1<<OSZIPIN);
      statusflag |= (1<<WAIT); // Flag setzen, Anzeigedauer beginnt
      
      ANZEIGE_HI_counter=0;
      ANZEIGE_LO_counter=0;
   }
   
}


/*
ISR (SPI_STC_vect) // Neue Zahl angekommen
{
   OSZI_B_LO;
   if (inindex==0)
   {
      //OSZI_B_LO;
      //OSZI_B_HI;
      //isrcontrol = spi_txbuffer[inindex] ;
   }
   isrcontrol++;
   spi_rxbuffer[inindex] = SPDR;
   //isrcontrol = inindex;
   //isrcontrol +=inindex;
   SPDR = spi_txbuffer[inindex];
   //uint8_t input = SPDR;
   
   spi_rxdata=1;
   //inindex = inc(&inindex);
   inindex++;
   //inindex &= 0x0F;
   //SPI_Data_counter++;
   OSZI_B_HI;
}
*/


void main (void) 
{
	
	wdt_disable();
	MCUSR &= ~(1<<WDRF);
	wdt_reset();
	WDTCR |= (1<<WDCE) | (1<<WDE);
	WDTCR = 0x00;
	slaveinit();
   
   MCUCR |= (1<<ISC01);
   GIMSK |= (1<<INT0);
   timer_init();
 //  timer1_init();
   sei();
   initADC(1);
   
 
	//Zaehler fuer Zeit von (SDA || SCL = LO)
	//uint16_t twi_LO_count0=0;
	//uint16_t twi_LO_count1=0;
	//uint8_t SlaveStatus=0x00; //status
    
	//Zaehler fuer Zeit von (SDA && SCL = HI)
	//uint16_t twi_HI_count0=0;
	
	//uint8_t twicontrol=0;
	
#pragma mark while
	while (1)
   {
      OCR0A = readKanal(1)>>2;
      //wdt_reset();
      //Blinkanzeige
      loopcount0++;
      if (loopcount0>=0x00F)
      {
         //lcd_gotoxy(0, 0);
         //lcd_putint(loopcount1);
         loopcount0=0;
         
         loopcount1++;
         if (loopcount1 >0x8F)
         {
            PORTB ^=(1<<LOOPLEDPIN);
            loopcount1=0;
            //           PORTB ^= (1<<RELAISPIN);
            
         }
         
      }
      
   }//while
   
   
    //return 0;
}
