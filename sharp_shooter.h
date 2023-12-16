#ifndef Sharp_shooter
#define Sharp_shooter

#define D0 (7)		// on port C
#define D1 (0)		// on port C
#define D2 (3)		// on port C
#define D3 (4)		// on port C
#define D4 (5)		// on port C
#define D5 (6)		// on port C
#define D6 (10)		// on port C
#define D7 (11)		// on port C

#define RS (12)		// on port C
#define E (13)		// on port C

#define BLUE_LED_POS (1)		// on port D
#define CONTROLLER_MOVE  (4)    //ptd
#define CONTROLLER_SHOOT   (4)    //pta

#define Buzzer_pin (1) //ptb tpm ch1
#define Buzzer_shoot (0) //pte
#define RESTART  (13) //pta

#define LCD_ENTRYLEFT    (0x02)
#define LCD_ENTRYMODESET  (0x04)
#define MASK(x) (1UL << (x))
#define MASK_R(X)   (1>>X)

//initializing peripherals
void Init_Pins(void);
void init_Timer();
void Init_Comparator(void);
void Control_B_LED(unsigned int blue_on);
void delay_ms(int t_ms);
void write_D0_D7(unsigned char instruction);

//LCD
void lcd_write_instruc (unsigned char instruction);
void lcd_write_char (unsigned char c);
void lcd_init (void);
void lcd_clear(void);
void lcd_goto(unsigned char column, unsigned char row);
void createChar(unsigned char *Pattern, const char Location);
void lcd_write_string(char *s);

//Game stuff
void Create_game_objects();
void Shoot();
void MOVE_player();
void Create_Enemy();
void Explosion_enemy();

//handlers
void PORTD_IRQHandler();
void TPM1_IRQHandler();
void PORTA_IRQHandler();

#endif
