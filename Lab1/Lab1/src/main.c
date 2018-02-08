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
int sendPotData = 0;
volatile avr32_adc_t *adc = &AVR32_ADC; // ADC IP registers address
unsigned short adc_channel_pot = 1;
unsigned short adc_channel_light = 2;


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
	dip204_set_cursor_position(1,1);
	dip204_hide_cursor();
	
}

void printLCDstring(char * data, int x, int y){
	dip204_set_cursor_position(x,y);
	dip204_write_string(data);
	dip204_set_cursor_position(1,1);
}

__attribute__((__interrupt__))
static void usart_int_handler(void)
{
	// Si cette interruption est lancee par une reception (bit RXRDY=1)
	if (AVR32_USART1.csr & (AVR32_USART_CSR_RXRDY_MASK))
	{
		//Lire le char recu dans registre RHR, et le stocker dans un 32bit
		char_recu = (AVR32_USART1.rhr & AVR32_USART_RHR_RXCHR_MASK);
	}
	else if(AVR32_USART1.csr & (AVR32_USART_CSR_TXRDY_MASK)){
		// get value for the potentiometer adc channel
		// Retransmettre un caractere vers le PC
		AVR32_USART1.idr = AVR32_USART_IDR_TXRDY_MASK;
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

__attribute__((__interrupt__))
static void adc_int_handler(){
	// Get value for the potentiometer adc channel
	AVR32_USART1.thr = (char)((adc_get_value(adc, adc_channel_pot) >> 2) & 0b11111110);
	//AVR32_USART1.thr = (char)((adc_get_value(adc, adc_channel_light) >> 2) | 0x01);
}

void init_pot(){
	adc_configure(&AVR32_ADC);
	adc_enable(&AVR32_ADC, adc_channel_pot);
	//adc_enable(&AVR32_ADC, adc_channel_light);
	INTC_register_interrupt(&adc_int_handler, AVR32_ADC_IRQ, AVR32_INTC_INT0);
	AVR32_ADC.ier = AVR32_ADC_IER_EOC1_MASK | AVR32_ADC_IER_EOC2_MASK;
}

void initialization(){
	char_recu = ' ';
	
	Disable_global_interrupt();
	INTC_init_interrupts();
	init_lcd();
	init_pot();
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);
	init_usart1();
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
			//adc_start(adc);
			AVR32_ADC.cr = AVR32_ADC_START_MASK;
		}
		else if(char_recu == 'x'){
			// stop
		}
	}
}