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
#include "usart.h"
#include "board.h"
#include "pm.h"
#include "adc.h"
#include "setup.h"

volatile avr32_tc_t *tc =  (&AVR32_TC);
volatile int SENSOR_LIGHT_HAS_VALUE = TRUE;
volatile int SENSOR_POT_HAS_VALUE = FALSE;
volatile int MODE_COURANT = 0;

//ADC
volatile int ADC_FRQ;
volatile int SPEED_UP_FRQ = FALSE;
volatile int aqcuisition = 0;
volatile U32 char_recu = ' ';


static const tc_waveform_opt_t WAVEFORM_OPT_TC0 =
{
	.channel  = TC_CHANNEL_0,                        // Channel selection.

	.bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
	.beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
	.bcpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOB.
	.bcpb     = TC_EVT_EFFECT_NOOP,                // RB compare effect on TIOB.

	.aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
	.aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
	.acpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOA: toggle.
	.acpa     = TC_EVT_EFFECT_TOGGLE,                // RA compare effect on TIOA: toggle
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

static const tc_waveform_opt_t WAVEFORM_OPT_TC1 =
{
	.channel  = TC_CHANNEL_1,                        // Channel selection.

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

static const tc_interrupt_t TC_INTERRUPT_TC0 =
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
static const tc_interrupt_t TC_INTERRUPT_TC1 =
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

/************************************************************************/
/* Interruption deu changement de                                       */
/************************************************************************/
__attribute__((__interrupt__))
static void tc_irq_1(){
	tc_read_sr(TC, TC_CHANNEL_1);

	LED_Toggle(LED5);
	
	if(char_recu == 's'){
		adc_start(&AVR32_ADC);

		aqcuisition = TRUE;
		
		// SURVIENT LORS D'UN DÉPASSEMENT DE CONVERSION DE L'ADC
		if(!adc_check_eoc(&AVR32_ADC, ADC_LIGHT_CHANNEL) || !adc_check_eoc(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL)){
			gpio_tgl_gpio_pin(LED2_GPIO);
		}
		
		// SURVIENT LORS D'UN DÉPASSEMENT DE FIN DE TRANSMISSION DE L'USART
		if (!(AVR32_USART1.csr & (AVR32_USART_CSR_TXRDY_MASK))){
			gpio_tgl_gpio_pin(LED3_GPIO);
		}
		
		if(SENSOR_POT_HAS_VALUE){
			AVR32_USART1.thr = (char)((adc_get_value(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL) >> 2) & ADC_POT_MASK);//Recuperation du canal
			SENSOR_LIGHT_HAS_VALUE = TRUE;
			SENSOR_POT_HAS_VALUE = FALSE;
		}
		
		if(SENSOR_LIGHT_HAS_VALUE){
			AVR32_USART1.thr = (char)((adc_get_value(&AVR32_ADC, ADC_LIGHT_CHANNEL) >> 2) | ADC_LIGHT_MASK);
			SENSOR_LIGHT_HAS_VALUE = FALSE;
			SENSOR_POT_HAS_VALUE = TRUE;
		}
	}
	else if(char_recu == 'x'){
		aqcuisition = FALSE;
		tc_stop(tc, TC_CHANNEL_1);
	}
}

/************************************************************************/
/* Interruption du boutton PBO                                          */
/************************************************************************/
__attribute__((__interrupt__))
static void toggle_bp0_handler(){


	if( gpio_get_pin_interrupt_flag( GPIO_PUSH_BUTTON_0 ) )
	{		// PB2 generated the interrupt.
		if(SPEED_UP_FRQ){
			ADC_FRQ *=  2;
			
			SPEED_UP_FRQ = FALSE;
			LED_On(LED5);
			}else{
			ADC_FRQ /=  2;
			SPEED_UP_FRQ = TRUE;
			LED_Off(LED5);

		}
		tc_stop(tc,TC_CHANNEL_1);
	
		tc_init_waveform(tc, &WAVEFORM_OPT_TC1);     // Initialize the timer/counter waveform.
		tc_write_rc(tc, TC_CHANNEL_1, ADC_FRQ); // Set RC value.
		tc_configure_interrupts(tc, TC_CHANNEL_1, &TC_INTERRUPT_TC1);
		tc_start(tc, TC_CHANNEL_1);                    // And start the timer/counter.
		// Clear the interrupt flag of the pin PB2 is mapped to.
		gpio_clear_pin_interrupt_flag(GPIO_PUSH_BUTTON_0);
	}
	
	
}


/************************************************************************/
/* Interruption des lumière                                             */
/************************************************************************/
__attribute__((__interrupt__))
static void tc_irq(void)
{
	// La lecture du registre SR efface le fanion de l'interruption.
	tc_read_sr(TC, TC_CHANNEL_0);

	// Toggle le LED1
	//gpio_tgl_gpio_pin(LED0_GPIO);
	LED_Toggle(LED0);
	
	//Toogle LED2 si en mode acquisition
	if(aqcuisition){
		//gpio_tgl_gpio_pin(LED1_GPIO);
		LED_Toggle(LED1);
	}
}



/************************************************************************/
/* Interruption du USART                                                */
/************************************************************************/
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


/************************************************************************/
/* Interruption du ADC pour l'acquisition de donnée                     */
/************************************************************************/
__attribute__((__interrupt__))
static void adc_int_handler(){
	//Retransmettre ce caractere vers le PC, si transmetteur disponible
	if (AVR32_USART1.csr & (AVR32_USART_CSR_TXRDY_MASK))
	{
		if(SENSOR_LIGHT_HAS_VALUE){
			AVR32_USART1.ier = AVR32_USART_IER_TXRDY_MASK;
		}
		
		if(SENSOR_POT_HAS_VALUE){
			AVR32_USART1.ier = AVR32_USART_IER_TXRDY_MASK;
		}
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
	init_dbg_rs232_ex(BAUDS_RATE,FOSC0);

	// Enregister le USART interrupt handler au INTC, level INT0
	INTC_register_interrupt(&usart_int_handler, AVR32_USART1_IRQ, AVR32_INTC_INT1);

	// Activer la source d'interrution du UART en reception (RXRDY)
	AVR32_USART1.ier = AVR32_USART_IER_RXRDY_MASK;
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
	adc_enable(&AVR32_ADC, ADC_LIGHT_CHANNEL);
	adc_enable(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL);
}

void init_tc_leds(){

	

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
	INTC_register_interrupt(&tc_irq, TC_IRQ0, AVR32_INTC_INT0);

  //Enable_global_interrupt();  // Active les interrupts

  tc_init_waveform(tc, &WAVEFORM_OPT_TC0);     // Initialize the timer/counter waveform.  
  // We want: (1/(fPBA/32)) * RC = 0.100 s, donc RC = (fPBA/32) / 10  to get an interrupt every 100 ms.
  tc_write_rc(tc, TC_CHANNEL_0, FRQ0 / 10); // Set RC value.
  tc_configure_interrupts(tc, TC_CHANNEL_0, &TC_INTERRUPT_TC0);
  tc_start(tc, TC_CHANNEL_0);                    // And start the timer/counter.


}

void init_tc_speed(){

	

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
  //INTC_init_interrupts();     // Initialise les vecteurs d'i.nterrupt

  // Enregistrement de la nouvelle IRQ du TIMER au Interrupt Controller .
	INTC_register_interrupt(&tc_irq_1, TC_IRQ1, AVR32_INTC_INT0);

  //Enable_global_interrupt();  // Active les interrupts

	ADC_FRQ = (ADC_DBL_FRQ /32)/10;
	tc_init_waveform(tc, &WAVEFORM_OPT_TC1);     // Initialize the timer/counter waveform.
	tc_write_rc(tc, TC_CHANNEL_1, ADC_FRQ); // Set RC value.
	tc_configure_interrupts(tc, TC_CHANNEL_1, &TC_INTERRUPT_TC1);
	tc_start(tc, TC_CHANNEL_1);                    // And start the timer/counter.

}

void initPBO(){
	//LE handler est appeler a chaque fois qu'un des 2 evenement est arrive
	gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_0 , GPIO_FALLING_EDGE);	// PB0
	gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_0 , GPIO_RISING_EDGE);	// PB0

	INTC_register_interrupt( &toggle_bp0_handler, AVR32_GPIO_IRQ_0 + (GPIO_PUSH_BUTTON_0/8), AVR32_INTC_INT2);


}

void initialization(){

	
	Disable_global_interrupt();
	INTC_init_interrupts();
	init_adc();
	init_usart1();
	init_tc_leds();
	init_tc_speed();
	initPBO();
	Enable_global_interrupt();
}
 
int main(void)
{
	initialization();
	
	
	while (TRUE)  
	{
	
		
	}
}