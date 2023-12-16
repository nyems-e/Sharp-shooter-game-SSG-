#include <mkl25z4.h>
#include "sharp_shooter.h"

void Init_Pins(void) {

	__disable_irq(); //disable interrupts writing to PRIMASK reg
	// Enable clock to ports
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK|SIM_SCGC5_PORTA_MASK|SIM_SCGC5_PORTB_MASK|SIM_SCGC5_PORTE_MASK;

	// GPIO pins
	PORTC->PCR[D0] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D0] |= PORT_PCR_MUX(1);
	PORTC->PCR[D1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D1] |= PORT_PCR_MUX(1);
	PORTC->PCR[D2] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D2] |= PORT_PCR_MUX(1);
	PORTC->PCR[D3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D3] |= PORT_PCR_MUX(1);
	PORTC->PCR[D4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D4] |= PORT_PCR_MUX(1);
	PORTC->PCR[D5] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D5] |= PORT_PCR_MUX(1);
	PORTC->PCR[D6] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D6] |= PORT_PCR_MUX(1);
	PORTC->PCR[D7] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D7] |= PORT_PCR_MUX(1);

	PORTD->PCR[CONTROLLER_MOVE] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[CONTROLLER_MOVE] |= PORT_PCR_MUX(1)|PORT_PCR_IRQC(10)|PORT_PCR_PE(1);//falling edge interrupt

	PORTA->PCR[CONTROLLER_SHOOT] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[CONTROLLER_SHOOT] |= PORT_PCR_MUX(1)|PORT_PCR_IRQC(10)|PORT_PCR_PE(1);//falling edge interrupt

	PORTA->PCR[RESTART] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[RESTART] |= PORT_PCR_MUX(1)|PORT_PCR_IRQC(10)|PORT_PCR_PE(1); //falling edge

	PORTB->PCR[Buzzer_pin ] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[Buzzer_pin] |= PORT_PCR_MUX(3); //TPM ch1
	//clearing flags
	PORTD->ISFR |= MASK(CONTROLLER_MOVE);
	PORTA->ISFR |= MASK(CONTROLLER_SHOOT);

	PORTC->PCR[RS] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RS] |= PORT_PCR_MUX(1);
	PORTC->PCR[E] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[E] |= PORT_PCR_MUX(1);

	PORTD->PCR[BLUE_LED_POS] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED_POS] |= PORT_PCR_MUX(1);

	PORTE->PCR[Buzzer_shoot] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[Buzzer_shoot] |= PORT_PCR_MUX(5); //CMP OUT


	// Set ports to outputs
	PTC->PDDR |= MASK(D0) | MASK(D1)  | MASK(D2)  | MASK(D3)  | MASK(D4)  | MASK(D5)  | MASK(D6)  | MASK(D7) ;
	PTC->PDDR |= MASK(RS) | MASK(E);
	PTD->PDDR |= MASK(BLUE_LED_POS);
	PTD->PDDR |= MASK(Buzzer_pin);

	//set ports as inputs
	PTD->PDDR &= ~MASK(CONTROLLER_MOVE);
	PTA->PDDR &= ~MASK(CONTROLLER_SHOOT);
	PTA->PDDR &= ~MASK(RESTART);

	NVIC_EnableIRQ(PORTA_IRQn); //shoot
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_SetPriority(PORTA_IRQn, 3);

	NVIC_EnableIRQ(PORTD_IRQn); //move
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	NVIC_SetPriority(PORTD_IRQn, 2);


	__enable_irq(); //enable interrupts
}


void init_Timer(){

	SIM->SCGC6 |=SIM_SCGC6_TPM1_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1) ;
	TPM1->MOD= 81920; //500ms
	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_MSA(1) |TPM_CnSC_ELSA(1); //output comapare for enemy movement
	TPM1->CONTROLS[1].CnSC |= TPM_CnSC_MSB(1) |TPM_CnSC_ELSA(1); //PWM low high for buzzer indicating game play has started

	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK| TPM_CnSC_CHIE_MASK; //ch0 for output compare , ch1 for PWM
	TPM1->CONTROLS[0].CnV |= 16000 ;
	TPM1->CONTROLS[1].CnV |= 15000 ;

	TPM1->SC |=  TPM_SC_TOF_MASK | TPM_SC_PS(7); //| TPM_SC_TOIE_MASK
	TPM1->SC |= TPM_SC_CMOD(1); //enable internal clock to run

	NVIC_EnableIRQ(TPM1_IRQn);
	NVIC_ClearPendingIRQ(TPM1_IRQn);
	NVIC_SetPriority(TPM1_IRQn, 4);
}

void Init_Comparator(void) {
	SIM->SCGC4 |= SIM_SCGC4_CMP_MASK; 	// Clock gate comparator
	CMP0->CR1 = CMP_CR1_EN_MASK | CMP_CR1_OPE_MASK; // Enable CMP, output pin on PTE0
	CMP0->CR1 |= CMP_CR1_INV_MASK ;		//inverting output player movement button goes low for movement
	//Plus select (Channel 4 PTE29), Minus Select (Channel 7- CMP DAC)
	CMP0->MUXCR = CMP_MUXCR_PSEL(5) | CMP_MUXCR_MSEL(7);
		// Enable DAC, reference voltage at 32/64 = 1/2 V
	CMP0->DACCR = CMP_DACCR_DACEN_MASK | CMP_DACCR_VOSEL(32);
}

void Control_B_LED(unsigned int blue_on) {
	if (blue_on) {
			PTD->PCOR = MASK(BLUE_LED_POS);
	}	else {
			PTD->PSOR = MASK(BLUE_LED_POS);
	}
}


//////////////////////////////////////////////////////////////////////////////
//---------code for LCD-------------------------------------------------------
const unsigned char     RS_Pin=RS; //0x01;
const unsigned char 	E_Pin=E; //x04;

void delay_ms(int t_ms){
	for (int i=0; i<t_ms; i++)
		for (int j=0; j<48000;j++);
}

void write_D0_D7(unsigned char instruction){
		(instruction & MASK(0) ) ?  	(PTC->PSOR = MASK(D0) ): (PTC->PCOR = MASK(D0) );
		(instruction & MASK(1) ) ?  	(PTC->PSOR = MASK(D1) ): (PTC->PCOR = MASK(D1) );
		(instruction & MASK(2) ) ?  	(PTC->PSOR = MASK(D2) ): (PTC->PCOR = MASK(D2) );
		(instruction & MASK(3) ) ?  	(PTC->PSOR = MASK(D3) ): (PTC->PCOR = MASK(D3) );
		(instruction & MASK(4) ) ?  	(PTC->PSOR = MASK(D4) ): (PTC->PCOR = MASK(D4) );
		(instruction & MASK(5) ) ?  	(PTC->PSOR = MASK(D5) ): (PTC->PCOR = MASK(D5) );
		(instruction & MASK(6) ) ?  	(PTC->PSOR = MASK(D6) ): (PTC->PCOR = MASK(D6) );
		(instruction & MASK(7) ) ?  	(PTC->PSOR = MASK(D7) ): (PTC->PCOR = MASK(D7) );

}

void lcd_write_instruc (unsigned char instruction)
	{
    	delay_ms(2);

		PTC->PCOR = MASK(RS);		//Set RS to 0 for instruction
        PTC->PCOR= MASK(E);			//set E to zero
		//set up instruction for D0 to D7
		write_D0_D7(instruction);

	    PTC->PSOR= MASK(E);			//set E to one
		PTC->PCOR= MASK(E);			//set E to zero to create falling edge
	}

void lcd_write_char (unsigned char c)
	{
		delay_ms(2);
		PTC->PSOR = MASK(RS);		//Set RS to 1 for data
        PTC->PCOR= MASK(E);			//set E to zero
		//set up instruction for D0 to D7
		write_D0_D7(c);
		PTC->PSOR= MASK(E);			//set E to one
		PTC->PCOR= MASK(E);			//set E to zero to create falling edge
	}

void lcd_init (void)
	{   //Table 11 of hitachi data sheet (8bit operation,8x2 line display using internal reset)
    	delay_ms(2);
    	lcd_write_instruc(0x06);        //Increment mode for the cursor (entry mode)
    	lcd_write_instruc(0x0C);        //The display on, the cursor off
    	lcd_write_instruc(0x38);        //An 8-bit data bus, two-line display
	}

void lcd_clear(void)
	{
	delay_ms(2);
	lcd_write_instruc(0x01);        //Clear the display
	lcd_write_instruc(0x02);        //returns the display to its original status if it was shifted.
}

void lcd_goto(unsigned char column, unsigned char row)
	{
	delay_ms(2);
	if(row==0)
	lcd_write_instruc(0x80 + column);
	if(row==1)
	lcd_write_instruc(0xC0+ column);
	}

void createChar(unsigned char *Pattern, const char Location){
	lcd_write_instruc(0x40+(Location*8)); //8 characters can generated
	for (int i=0; i<8; i++) {
			lcd_write_char(Pattern[i]);// filling characters for each CGRAM block of memory
		}
}

void lcd_write_string(char *s)
	{
	delay_ms(2);
	while(*s != 0)
	{
	lcd_write_char(*s);
	s++;
	}
	}


//Character patterns for CGRAM 
unsigned char Enemy[] = {0x00,0x15,0x0E,0x0A,0x0E,0x15,0x00,0x00};
unsigned char Explode[] = {0x00,0x0A,0x15,0x0A,0x15,0x0A,0x00,0x00};
unsigned char Explode1[] = {0x00,0x00,0x00,0x11,0x04,0x04,0x01,0x11};
unsigned char Clear[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char Player[] = {0x0E,0x0E,0x05,0x06,0x05,0x04,0x0A,0x0A};
unsigned char Player1[] = {0x0E,0x0E,0x04,0x05,0x06,0x05,0x0E,0x11};
unsigned char Bullet[] = {0x00,0x00,0x00,0x00,0x00,0x1F,0x00,0x00};


//global variables for game play
static volatile unsigned int g_Move_player  = 0;
static volatile unsigned int g_enemy = 0;
static volatile unsigned int g_bullet = 0;
static volatile unsigned int g_v_position_player = 0;
static volatile unsigned int g_bullet_position_x = 0;
static volatile unsigned int g_bullet_position_y = 0;
static volatile unsigned int g_enemy_position_x = 1;
static volatile unsigned int g_enemy_position_y = 1;
static volatile unsigned int enemy_pos = 15;




// Creates all the game objects
void Create_game_objects(){
	createChar(Clear,2);
	createChar(Enemy,1);
	createChar(Explode,4);
	createChar(Explode1,5);
	createChar(Player,0);
	createChar(Bullet,3);
	createChar(Player1,6);
}

//shooting action based on PORT A interrupt
void Shoot(){
	if (g_bullet ==1){
		PTD->PCOR = MASK(BLUE_LED_POS);
		g_bullet =0;
		enum S_states {S1,S2,S3,S4,S5};
		static enum S_states next_state_1=S1;
		if(g_v_position_player){

		switch(next_state_1){
		case S1:
			lcd_goto(1,1);
			lcd_write_char(3);
			g_bullet_position_x = 1;
			g_bullet_position_y = 1;

			//clear
			lcd_goto(1,1);
			lcd_write_char(2);
			next_state_1 =S2;
			break;
			case S2:
				lcd_goto(2,1);
				lcd_write_char(3);
				g_bullet_position_x = 2;
				g_bullet_position_y = 1;
				//clear
				lcd_goto(2,1);
				lcd_write_char(2);
				next_state_1 =S3;
			break;
			case S3:

					lcd_goto(3,1);
					lcd_write_char(3);
					g_bullet_position_x = 3;
					g_bullet_position_x = 1;
					//clear
					lcd_goto(3,1);
					lcd_write_char(2);
					next_state_1 =S4;

				break;

				case S4:
					lcd_goto(4,1);
					lcd_write_char(3);
					g_bullet_position_x = 4;
					g_bullet_position_y = 1;
					//clear
					lcd_goto(4,1);
					lcd_write_char(2);
					next_state_1 =S5;

					break;
			case S5:
				lcd_goto(5,1);
				lcd_write_char(3);
				g_bullet_position_x = 5;
				g_bullet_position_y = 1;
				//clear
				lcd_goto(5,1);
				lcd_write_char(2);

				next_state_1 =S1;

				break;
		}
		}else{
			enum S_states {S1,S2,S3,S4,S5};
			static enum S_states next_state=S1;

			switch(next_state){
					case S1:

						lcd_goto(1,0);
						lcd_write_char(3);
						g_bullet_position_x = 1;
						g_bullet_position_y = 1;
						lcd_goto(1,0);
						lcd_write_char(2);
						next_state =S2;
						break;
				case S2:
						lcd_goto(2,0);
						lcd_write_char(3);
						g_bullet_position_x = 2;
						g_bullet_position_y = 1;
						//clear
						lcd_goto(2,0);
						lcd_write_char(2);
						next_state =S3;
						break;
					case S3:

						lcd_goto(3,0);
						lcd_write_char(3);
						g_bullet_position_x = 3;
						g_bullet_position_y = 1;
						//clear
						lcd_goto(3,0);
						lcd_write_char(2);
						next_state =S4;

						break;

					case S4:
						lcd_goto(4,0);
						lcd_write_char(3);
						g_bullet_position_x = 4;
						g_bullet_position_y = 1;
						//clear
						lcd_goto(4,0);
						lcd_write_char(2);
						next_state =S5;

						break;
					case S5:
						lcd_goto(5,0);
						lcd_write_char(3);
						g_bullet_position_x = 5;
						g_bullet_position_y = 1;
						//clear
						lcd_goto(5,0);
						lcd_write_char(2);

						next_state =S1;

						break;

		}

		}
	}
	}


//move player based on PORTD interrupt
void MOVE_player(){
	enum P_states {UP, DOWN};
	static enum P_states next_state=DOWN;
	if(g_Move_player){
		g_Move_player = 0;
	switch(next_state){
		case UP:
			//clear
			lcd_goto(0,1);
			lcd_write_char(2);

            //Player
			lcd_goto(0,0);
			lcd_write_char(0);
			lcd_goto(0,0);
			lcd_write_char(6); //swing arms
			next_state = DOWN;
			break;
		case DOWN:
			//clear
			lcd_goto(0,0);
			lcd_write_char(2);
			//Player
			lcd_goto(0,1);
			lcd_write_char(0);
			lcd_goto(0,1);
			lcd_write_char(6);
			next_state = UP;
			break;
		default:
			next_state = DOWN;
			break;
	}
	}
}

//create enemy based TPM output compare interrupt
void Create_Enemy(){
	enum E_states {UP, DOWN};
	static enum E_states next_state=DOWN;

	if(g_enemy == 1 && enemy_pos > 0){
			g_enemy = 0;
		switch(next_state){
			case UP:
				//clear
				lcd_goto(enemy_pos,1);
				lcd_write_char(2);
	            //Enemy
				lcd_goto(enemy_pos,0);
				lcd_write_char(1);
				g_enemy_position_x = enemy_pos;
				g_enemy_position_y = 0;

				lcd_goto(enemy_pos,0);
				lcd_write_char(2);
				enemy_pos--;
				next_state = DOWN;
				break;
			case DOWN:
				//clear
				lcd_goto(enemy_pos,0);
				lcd_write_char(2);
				//Enemy
				lcd_goto(enemy_pos,1);
				lcd_write_char(1);
				g_enemy_position_x = enemy_pos;
				g_enemy_position_y = 1;

				next_state = UP;
				break;
			default:
				next_state = DOWN;
				break;
		}
		} else {
			lcd_clear();
			lcd_write_string("GAME ENDED");

		}
}

//creates explosion animation
void Explosion_enemy(){
	if(g_enemy_position_x == g_bullet_position_x && g_enemy_position_y == g_bullet_position_y){
		lcd_goto(g_bullet_position_x,g_bullet_position_y);
		lcd_write_char(4);
		lcd_goto(g_bullet_position_x,g_bullet_position_y);
		lcd_write_char(5);
		lcd_goto(g_bullet_position_x,g_bullet_position_y);
		lcd_write_char(2);
			}

}


// handlers
void PORTD_IRQHandler(){
	if(PORTD->ISFR & MASK(CONTROLLER_MOVE)){
		g_Move_player = 1;
		g_v_position_player ^= MASK(0);
	PORTD->ISFR |= MASK(CONTROLLER_MOVE);

	}
}


void TPM1_IRQHandler(){
	if (TPM1->STATUS & TPM_STATUS_CH0F_MASK){
		g_enemy = 1;
		TPM1->CONTROLS[0].CnSC |=TPM_CnSC_CHF_MASK;
		TPM1->CONTROLS[1].CnSC |=TPM_CnSC_CHF_MASK;
		}

}

void PORTA_IRQHandler(){
	if(PORTA->ISFR & MASK(CONTROLLER_SHOOT )){
		g_bullet = 1;
		PTD->PSOR = MASK(BLUE_LED_POS);
	PORTA->ISFR |= MASK(CONTROLLER_SHOOT );
	}
}









