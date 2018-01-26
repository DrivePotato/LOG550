/*=============================================================================*/
/* 
	lab1.c
	Prototype 1
	Description:
	Composant:
*/
/*=============================================================================*/

#include <asf.h>   
#include "compiler.h" // Definitions utile: de U8, S8, U16, S16, U32, S32, U32, U62, F32
#include "gpio.h"
#include "pm.h"
#include "adc.h"
#define   TRUE   1
#define   FALSE   0


volatile U32 char_recu; 
U32 value_pot = 0;
int sendPotData = 0;
volatile avr32_adc_t *adc = &AVR32_ADC; // ADC IP registers address
unsigned short adc_channel_pot = 1;


void init_lcd(){
	// Assign I/Os to SPI
	static const gpio_map_t DIP204_SPI_GPIO_MAP =
	{
		{DIP204_SPI_SCK_PIN,  DIP204_SPI_SCK_FUNCTION },  // SPI Clock.
		{DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION},  // MISO.
		{DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION},  // MOSI.
		{DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
	};
	
	
	// add the spi options driver structure for the LCD DIP204
	spi_options_t spiOptions =
	{
		.reg          = DIP204_SPI_NPCS,
		.baudrate     = 1000000,
		.bits         = 8,
		.spck_delay   = 0,
		.trans_delay  = 0,
		.stay_act     = 1,
		.spi_mode     = 0,
		.modfdis      = 1
	};
	
	
	gpio_enable_module(DIP204_SPI_GPIO_MAP,
	sizeof(DIP204_SPI_GPIO_MAP) / sizeof(DIP204_SPI_GPIO_MAP[0]));

	// Initialize as master
	spi_initMaster(DIP204_SPI, &spiOptions);

	// Set selection mode: variable_ps, pcs_decode, delay
	spi_selectionMode(DIP204_SPI, 0, 0, 0);

	// Enable SPI
	spi_enable(DIP204_SPI);

	// setup chip registers
	spi_setupChipReg(DIP204_SPI, &spiOptions, FOSC0);

	// initialize delay driver
	delay_init( FOSC0 );

	// initialize LCD
	dip204_init(backlight_PWM, TRUE);	
}

void printLCD(U32 data, int x, int y){
	dip204_set_cursor_position(x,y);
	dip204_write_data(data);
	dip204_set_cursor_position(x,y);
}

void printLCDstring(char * data, int x, int y){
	dip204_set_cursor_position(x,y);
	dip204_write_string(data);
	dip204_set_cursor_position(x,y);
}

__attribute__((__interrupt__))
static void usart_int_handler(void)
{
	//      bit TXRDY : Ce bit se leve lorsqu'un transmission (vers le PC se termine,
	//                  et demeure lever tant que le transmetteur est disponible.
	//                  Si on lance une trasmission, celui-ci descend le temps de transmettre.
	//                  Attention, lorsque le transmetteur ne transmet pas, ce bit est toujours a 1,
	//                  donc il va toujours relancer l'interruption si vous oubliez le bit TXRDY du IER.
	
	
	// Si cette interruption est lancee par une reception (bit RXRDY=1)
	if (AVR32_USART1.csr & (AVR32_USART_CSR_RXRDY_MASK))
	{
		//Lire le char recu dans registre RHR, et le stocker dans un 32bit
		char_recu = (AVR32_USART1.rhr & AVR32_USART_RHR_RXCHR_MASK);
		//Eliminer la source de l'IRQ, bit RXRDY (automatiquement mis a zero a la lecture de RHR)
	}
	else{
		// get value for the potentiometer adc channel
		// Retransmettre un caractere vers le PC
		printLCDstring("Pot:", 1, 3);
		printLCD(value_pot, 5, 3);
		
		adc->cr = AVR32_ADC_START_MASK;
		AVR32_USART1.thr = value_pot & ~0x01 ;
		//AVR32_USART1.ier = AVR32_USART_IER_TXRDY_MASK;
		//AVR32_USART1.thr = AVR32_ADC.cdr1 | 0x01;
		//AVR32_USART1.idr = AVR32_USART_IDR_TXRDY_MASK;

		sendPotData = TRUE;		
	}
}

void init_usart1(){
	static const gpio_map_t USART_GPIO_MAP =
	{
		{AVR32_USART1_RXD_0_0_PIN, AVR32_USART1_RXD_0_0_FUNCTION},
		{AVR32_USART1_TXD_0_0_PIN, AVR32_USART1_TXD_0_0_FUNCTION}
	};

	// Assigner les pins du GPIO a etre utiliser par le USART1.
	gpio_enable_module(USART_GPIO_MAP,sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

	// Initialise le USART1 en mode seriel RS232, a 57600 BAUDS, a FOSC0=12MHz.
	init_dbg_rs232_ex(57600,FOSC0);

	// Enregister le USART interrupt handler au INTC, level INT0
	INTC_register_interrupt(&usart_int_handler, AVR32_USART1_IRQ, AVR32_INTC_INT0);

	// Activer la source d'interrution du UART en reception (RXRDY)
	AVR32_USART1.ier = AVR32_USART_IER_RXRDY_MASK;
}

// code sur internet
__attribute__((__interrupt__))
static void adc_int_handler(void)
{
	printLCDstring("ADC INT", 1, 3);
}

void init_pot(){
	static const gpio_map_t ADC_GPIO_MAP =
	{
		{ADC_POTENTIOMETER_PIN, ADC_POTENTIOMETER_FUNCTION}
	};

	volatile avr32_adc_t *adc = &AVR32_ADC; // ADC IP registers address
	signed short adc_value_pot = -1;

	// Assign the on-board sensors to their ADC channel.
	unsigned short adc_channel_pot = ADC_POTENTIOMETER_CHANNEL;

	// switch to oscillator 0
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);

	// init debug serial line
	init_dbg_rs232(FOSC0);

	// Assign and enable GPIO pins to the ADC function.
	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) / sizeof(ADC_GPIO_MAP[0]));

	// configure ADC
	// Lower the ADC clock to match the ADC characteristics (because we configured
	// the CPU clock to 12MHz, and the ADC clock characteristics are usually lower;
	// cf. the ADC Characteristic section in the datasheet).
	AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;
	adc_configure(adc);

	// Enable the ADC channels.
	adc_enable(adc,adc_channel_pot);
}

void initialization(){
	char_recu = ' ';
	
	// Init le LCD avant de switch to osc0
	init_lcd();
	
	// Desactive les interruptions pendant la configuration.
	Disable_global_interrupt();
	// WARNING: NE PEUT PLUS PRINT AU LCD AVEC LE SWITCH FUNCTION, Au boot, 115kHz, on doit passer au crystal FOSC0=12MHz avec le PM
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);
	// Preparatif pour l'enregistrement des interrupt handler du INTC.
	INTC_init_interrupts();
	init_pot();
	init_usart1();
	
	// Autoriser les interruptions.
	Enable_global_interrupt();
}
 
int main(void)
{
	initialization();
	
	printLCDstring("Prototype 1", 1, 1);
	printLCDstring("Recu: ", 1, 2);
	
	while (TRUE)  
	{
		printLCD(char_recu, 7, 2);
		if(char_recu == 's'){
			// Get value from adc
			//char value = (adc_get_value(adc, adc_channel_pot) << 7) | 0x01;
			//printLCDstring(adc_get_value(adc, adc_channel_pot), 1, 3);
			// send light value
		}
		else if(char_recu == 'x'){
			// stop
		}
		
		// launch conversion on all enabled channels
		adc_start(adc);
		// get value for the potentiometer adc channel
		printLCD(adc_get_value(adc, adc_channel_pot), 1, 3);
		// display value to user
	}
}