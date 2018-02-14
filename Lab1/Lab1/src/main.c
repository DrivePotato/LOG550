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

/*   Timer counter constant */
#  define TC_CHANNEL					0
#  define TC							(&AVR32_TC)
#  define TC_IRQ_GROUP					AVR32_TC_IRQ_GROUP
#  define TC_IRQ0						AVR32_TC_IRQ0
#  define FPBA							FOSC0          // FOSC0 est a 12Mhz
# define BASE_FREQ                      1000
# define BPIO_FREQ                      2000

# define MODE1							TRUE
# define ADC_SPEED1						1000
# define ADC_SPEED2						2000

volatile avr32_tc_t *tc =  (&AVR32_TC);
volatile U32 char_recu; 
volatile int SENSOR_LIGHT_HAS_VALUE;
volatile int SENSOR_POT_HAS_VALUE;
volatile int aqcuisition = 0;

__attribute__((__interrupt__))
static void tc_irq(void)
{
	// La lecture du registre SR efface le fanion de l'interruption.
	tc_read_sr(TC, TC_CHANNEL);

	// Toggle le un LED
	gpio_tgl_gpio_pin(LED0_GPIO);

	//Toogle LED2 si en mode acquisition
	if(aqcuisition){
		gpio_tgl_gpio_pin(LED1_GPIO);
	}
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
	INTC_register_interrupt(&usart_int_handler, AVR32_USART1_IRQ, AVR32_INTC_INT1);

	// Activer la source d'interrution du UART en reception (RXRDY)
	AVR32_USART1.ier = AVR32_USART_IER_RXRDY_MASK;
}

__attribute__((__interrupt__))
static void adc_int_handler(){
	// Get value for the potentiometer adc channel
	//AVR32_USART1.thr = (char)((adc_get_value(adc, adc_channel_pot) >> 2) & 0b11111110);
	//AVR32_USART1.thr = (char)((adc_get_value(adc, adc_channel_light) >> 2) | 0x01);
	//if(AVR32_USART1.csr & (AVR32_USART_CSR_TXRDY_MASK)){
		//if(AVR32_ADC.sr & AVR32_ADC_SR_EOC1_MASK){
			//AVR32_USART1.thr = (char)((adc_get_value(&AVR32_ADC, adc_channel_pot) >> 2) & 0b11111110);
				//AVR32_USART1.thr = (char)((AVR32_ADC.cdr1 >> 2) & 0b11111110); // POT
			 
			//SENSOR_POT_HAS_VALUE = TRUE;
		//	SENSOR_LIGHT_HAS_VALUE = TRUE;
		//}
	//}
	//if(adc->sr & AVR32_ADC_SR_EOC2_MASK){
	//	sensorPotValue = adc_get_value(adc, adc_channel_pot);	
	//	SENSOR_LIGHT_HAS_VALUE = FALSE;
	//}
	
	
	
	
	//Retransmettre ce caractere vers le PC, si transmetteur disponible, renvoi un echo
	if (AVR32_USART1.csr & (AVR32_USART_CSR_TXRDY_MASK))
	{
		if(SENSOR_LIGHT_HAS_VALUE){
			if ( AVR32_ADC.sr & AVR32_ADC_SR_EOC2_MASK ){
				AVR32_USART1.thr =(char)((adc_get_value(&AVR32_ADC, ADC_LIGHT_CHANNEL) >> 2) | 0x01); // & AVR32_USART_THR_TXCHR_MASK on renvoi le char
				// Activer la source d'interrution du UART en fin de transmission (TXRDY)
				AVR32_USART1.ier = AVR32_USART_IER_TXRDY_MASK;
			}
		}
		
		if(SENSOR_POT_HAS_VALUE){
			//AVR32_USART1.thr = (char)((adc_get_value(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL) >> 2) & 0b11111110) ; 
			// Activer la source d'interrution du UART en fin de transmission (TXRDY)
			AVR32_USART1.ier = AVR32_USART_IER_TXRDY_MASK;
		}
	}
}

void init_adc(){
	const gpio_map_t ADC_GPIO_MAP = {
		{ADC_LIGHT_PIN, ADC_LIGHT_FUNCTION},
		{ADC_POTENTIOMETER_PIN, ADC_POTENTIOMETER_FUNCTION}
	};
	
	/* Assign and enable GPIO pins to the ADC function. */
	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) /
	sizeof(ADC_GPIO_MAP[0]));
	/* Configure the ADC peripheral module.
	 * Lower the ADC clock to match the ADC characteristics (because we
	 * configured the CPU clock to 12MHz, and the ADC clock characteristics are
	 *  usually lower; cf. the ADC Characteristic section in the datasheet). */
	AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;
	adc_configure(&AVR32_ADC);
	adc_enable(&AVR32_ADC, ADC_LIGHT_CHANNEL); //PROBLEME EST QUAND ON ENABLE LE LIGHT SENSOR ...
	adc_enable(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL);
}

void init_tc(){

	static const tc_waveform_opt_t WAVEFORM_OPT =
	{
		.channel  = TC_CHANNEL,                        // Channel selection.

		.bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
		.beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
		.bcpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOB.
		.bcpb     = TC_EVT_EFFECT_NOOP,                // RB compare effect on TIOB.

		.aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
		.aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
		.acpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOA: toggle.
		.acpa     = TC_EVT_EFFECT_NOOP,                // RA compare effect on TIOA: toggle
		.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,// Waveform selection: Up mode with automatic trigger(reset) on RC compare.
		.enetrg   = FALSE,                             // External event trigger enable.
		.eevt     = 0,                                 // External event selection.
		.eevtedg  = TC_SEL_NO_EDGE,                    // External event edge selection.
		.cpcdis   = FALSE,                             // Counter disable when RC compare.
		.cpcstop  = FALSE,                             // Counter clock stopped with RC compare.

		.burst    = FALSE,                             // Burst signal selection.
		.clki     = FALSE,                             // Clock inversion.
		.tcclks   = TC_CLOCK_SOURCE_TC4                // Internal source clock 3, connected to fPBA / 8.
	};
	
	

	static const tc_interrupt_t TC_INTERRUPT =
	{
		.etrgs = 0,
		.ldrbs = 0,
		.ldras = 0,
		.cpcs  = 1,
		.cpbs  = 0,
		.cpas  = 0,
		.lovrs = 0,
		.covfs = 0
	};


	  /*! \brief Main function:
   *  - Configure the CPU to run at 12MHz
   *  - Register the TC interrupt (GCC only)
   *  - Configure, enable the CPCS (RC compare match) interrupt, and start a TC channel in waveform mode
   *  - In an infinite loop, do nothing
   */

  /* Au reset, le microcontroleur opere sur un crystal interne a 115200Hz. */
  /* Nous allons le configurer pour utiliser un crystal externe, FOSC0, a 12Mhz. */
  pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);

  //Disable_global_interrupt(); // Desactive les interrupts le temps de la config
  //INTC_init_interrupts();     // Initialise les vecteurs d'interrupt

  // Enregistrement de la nouvelle IRQ du TIMER au Interrupt Controller .
  INTC_register_interrupt(&tc_irq, TC_IRQ0, AVR32_INTC_INT2);
  //Enable_global_interrupt();  // Active les interrupts

  tc_init_waveform(tc, &WAVEFORM_OPT);     // Initialize the timer/counter waveform.

  // Placons le niveau RC a atteindre pour declencher de l'IRQ.
  // Attention, RC est un 16-bits, valeur max 65535

  // We want: (1/(fPBA/32)) * RC = 0.100 s, donc RC = (fPBA/32) / 10  to get an interrupt every 100 ms.
  tc_write_rc(tc, TC_CHANNEL, (FPBA / 32) / 10); // Set RC value.

  tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);

  // Start the timer/counter.
  tc_start(tc, TC_CHANNEL);                    // And start the timer/counter.

}
__attribute__((__interrupt__))
static void toggle_bp0_handler(){

		
		 if( gpio_get_pin_interrupt_flag( AVR32_PIN_PB00 ) )
		 {		
			 	gpio_tgl_gpio_pin(LED2_GPIO);

			 // Clear the interrupt flag of the pin PB2 is mapped to.
			 gpio_clear_pin_interrupt_flag(AVR32_PIN_PB00);
		 }
		
}
void initPBO(){
	gpio_enable_module_pin(AVR32_PIN_PB00,&toggle_bp0_handler);
	gpio_enable_pin_interrupt(AVR32_PIN_PB00, GPIO_PIN_CHANGE);
	INTC_register_interrupt( &toggle_bp0_handler, AVR_PI, AVR32_INTC_INT3);

}

void initialization(){
	char_recu = ' ';
	
	Disable_global_interrupt();
	INTC_init_interrupts();
	init_adc();
	init_usart1();
	init_tc();
	initPBO();
	Enable_global_interrupt();
}
 
int main(void)
{
	initialization();
	
	SENSOR_LIGHT_HAS_VALUE = FALSE;
	SENSOR_POT_HAS_VALUE = TRUE;
	while (TRUE)  
	{
		if(char_recu == 's'){
			adc_start(&AVR32_ADC);
			aqcuisition = TRUE;
			
			if(SENSOR_LIGHT_HAS_VALUE){
				AVR32_USART1.thr = (char)((adc_get_value(&AVR32_ADC, ADC_LIGHT_CHANNEL) >> 2) | 0x01);	
				SENSOR_LIGHT_HAS_VALUE = FALSE;
				SENSOR_POT_HAS_VALUE = TRUE;
			}
			else if(SENSOR_POT_HAS_VALUE){
				AVR32_USART1.thr = (char)((adc_get_value(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL) >> 2) & 0b11111110);
				SENSOR_LIGHT_HAS_VALUE = TRUE;
				SENSOR_POT_HAS_VALUE = FALSE;
			}
			
			AVR32_USART1.idr = AVR32_USART_IDR_TXRDY_MASK;
		}
		else if(char_recu == 'x'){
			aqcuisition = FALSE;
		}
		
	
				
		
		

	}
}