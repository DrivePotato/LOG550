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
#include "tc.h"
#define   TRUE   1
#define   FALSE   0

volatile int *aqcuisition = 0;
U8 u8LedMap=0x01;
volatile U32 i , k;
volatile U32 char_recu; 
int sendPotData = 0;
volatile avr32_adc_t *adc = &AVR32_ADC; // ADC IP registers address
unsigned short adc_channel_pot = 1;

volatile avr32_tc_t *tc = EXAMPLE_TC;

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

static void tc_irq(void)
{
	// La lecture du registre SR efface le fanion de l'interruption.
	tc_read_sr(EXAMPLE_TC, TC_CHANNEL);

	// Toggle le premier et le second LED.
	gpio_tgl_gpio_pin(LED0_GPIO);
	
	//Toogle LED2 si en mode acquisition
	if(*aqcuisition){
		gpio_tgl_gpio_pin(LED1_GPIO);
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

void init_tc(){
	  /*! \brief Main function:
   *  - Configure the CPU to run at 12MHz
   *  - Register the TC interrupt (GCC only)
   *  - Configure, enable the CPCS (RC compare match) interrupt, and start a TC channel in waveform mode
   *  - In an infinite loop, do nothing
   */

  /* Au reset, le microcontroleur opere sur un crystal interne a 115200Hz. */
  /* Nous allons le configurer pour utiliser un crystal externe, FOSC0, a 12Mhz. */
  pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);

  Disable_global_interrupt(); // Desactive les interrupts le temps de la config
  INTC_init_interrupts();     // Initialise les vecteurs d'interrupt

  // Enregistrement de la nouvelle IRQ du TIMER au Interrupt Controller .
  INTC_register_interrupt(&tc_irq, EXAMPLE_TC_IRQ, AVR32_INTC_INT0);
  Enable_global_interrupt();  // Active les interrupts

  tc_init_waveform(tc, &WAVEFORM_OPT);     // Initialize the timer/counter waveform.

  // Placons le niveau RC a atteindre pour declencher de l'IRQ.
  // Attention, RC est un 16-bits, valeur max 65535

  // We want: (1/(fPBA/32)) * RC = 0.100 s, donc RC = (fPBA/32) / 10  to get an interrupt every 100 ms.
  tc_write_rc(tc, TC_CHANNEL, (FPBA / 32) / 10); // Set RC value.

  tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);

  // Start the timer/counter.
  tc_start(tc, TC_CHANNEL);                    // And start the timer/counter.

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
	init_tc()
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
		//Activer LED1
		LED_Display(u8LedMap);
		
		
		
		printLCD(char_recu, 7, 2);
		if(char_recu == 's'){
			adc_start(adc);
			// Get value for the potentiometer adc channel
			AVR32_USART1.thr = ((adc_get_value(adc, adc_channel_pot) >> 2) & 0b11111110);//& AVR32_USART_THR_TXCHR_MASK;
			*aqcuisition = TRUE;
			
			
		}
		else if(char_recu == 'x'){
			// stop
			*aqcuisition = FALSE;

		}
		
		
		// TODO Allumer LED3 si depassement au convertisseur ADC
		
		
		// TODO Allumer LED3 si depassement au convertisseur UART
		
		
	}
}