#include <mkl25z4.h>
#include "sharp_shooter.h"

int main(void)
{	PTD->PCOR = MASK(BLUE_LED_POS);
	lcd_clear();
	Init_Pins();
	lcd_init();
	init_Timer();
	Init_Comparator();

	PTD->PSOR = MASK(BLUE_LED_POS);
	Create_game_objects();

	lcd_goto(0,0); //create player
	lcd_write_char(0); //create player
	lcd_goto(0,0);
	lcd_write_char(6); //swing player arms
	PTD->PSOR = MASK(BLUE_LED_POS);

	PTD->PCOR = MASK(BLUE_LED_POS);


	 while(1){
		 MOVE_player();
		 Create_Enemy();
		 Shoot();
		 Explosion_enemy();
	 }
}

