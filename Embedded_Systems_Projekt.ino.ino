#define F_CPU 16000000ul;
#define HC595_PORT PORTB
#define HC595_DDR DDRB
#define HC595_DS_POS PB0     //Data pin (DS) pin location
#define HC595_SH_CP_POS PB1  //Shift Clock (SH_CP) pin location
#define HC595_ST_CP_POS PB2  //Store Clock (ST_CP) pin location




// change data (DS)lines
#define HC595DataHigh() (HC595_PORT |= (1 << HC595_DS_POS))
#define HC595DataLow() (HC595_PORT &= (~(1 << HC595_DS_POS)))

#include "lcd.h"
#include <avr/io.h>
#include <util/delay.h>

void shiftInit() {
  //Make the Data(DS), Shift clock (SH_CP), Store Clock (ST_CP) lines output
  HC595_DDR |= ((1 << HC595_SH_CP_POS) | (1 << HC595_ST_CP_POS) | (1 << HC595_DS_POS));
}

//Sends a clock pulse on SH_CP line
void shiftPulse() {
  //Pulse the Shift Clock
  HC595_PORT |= (1 << HC595_SH_CP_POS);     //HIGH
  HC595_PORT &= (~(1 << HC595_SH_CP_POS));  //LOW
}
//Sends a clock pulse on ST_CP line
void shiftLatch() {
  //Pulse the Store Clock
  HC595_PORT |= (1 << HC595_ST_CP_POS);  //HIGH
  _delay_loop_1(1);
  HC595_PORT &= (~(1 << HC595_ST_CP_POS));  //LOW
  _delay_loop_1(1);
}

void shiftWrite(uint8_t data) {
  //Send each 8 bits serially
  //Order is MSB first
  for (uint8_t i = 0; i < 8; i++) {
    //Output the data on DS line according to the
    //Value of MSB
    if (data & 0b10000000) {
      //MSB is 1 so output high
      HC595DataHigh();
    } else {
      //MSB is 0 so output high
      HC595DataLow();
    }
    shiftPulse();      //Pulse the Clock line
    data = data << 1;  //Now bring next bit at MSB position
    _delay_ms(5);
  }
  //Now all 8 bits have been transferred to shift register
  //Move them to output latch at one
  shiftLatch();
}

void Wait() {
  for (uint8_t i = 0; i < 50; i++) {
    _delay_loop_2(0);
  }
}

long entfernung;

int main(void) {
  Serial.begin(9600);

  lcd_init(LCD_DISP_ON);

  uint8_t led[9] = {
    0b00000000,
    0b10000000,
    0b11000000,
    0b11100000,
    0b11110000,
    0b11111000,
    0b11111100,
    0b11111110,
    0b11111111
  };
  shiftInit();  //Initialise

  // DDR als Ausgang setzen -> Ultraschallsensor (Trigger- und Echo-Pin)
  DDRD = 0xff;
  // DDR als Ausgang setzen -> LED
  DDRH = 0xff;
  // DDR als Ausgang setzen -> Motor
  DDRA = 0xff;

  TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0B1);
  TCCR0B |= (1 << CS01);

  DDRG |= (1 << DDD5);

  // DDR am Echo-Pin (Pin3) als Eingang setzen
  DDRD &= ~(1 << PD3);
  // Pull-up Widerstand am Echo-Pin einstellen
  PORTD |= (1 << PD3);

  while (1) {
    // ggf. Trigger manuell beenden

    // Ultraschallsensor sendet Signal los -> Trigger-Pin auf HIGH
    PORTD |= (1 << PD2);
    // Signal vom Trigger sendet 10 Millisekunden lang
    _delay_ms(10);
    // Signal vom Trigger sendet nicht mehr
    PORTD &= ~(1 << PD2);

    // warten auf eingehenden Signal beim Echo
    while (!(PIND & (1 << PD3))) {
      // do nothn
    }

    // Signal kommt beim Echo an
    while (PIND & (1 << PD3)) {
      entfernung = entfernung + 1;
      _delay_us(58);
    }

    // Distance auf Oled
    lcd_gotoxy(0,2);
    lcd_puts_p(PSTR("Entfernung in cm: "));
    lcd_gotoxy(30,2);
    if(entfernung<100){
      lcd_gotoxy(33,2);
      lcd_puts(" ");
    }
    if(entfernung<10){
      lcd_gotoxy(32,2);
      lcd_puts(" ");
    }

    char int_str[20];
    itoa(entfernung,int_str,10);
    
    lcd_puts(int_str);

    // Balken-Anzeige
    uint8_t i;
    uint8_t j;

    // Motor PWM

    if (entfernung <= 10) {
      shiftWrite(led[8]);
      OCR0B = 0;  // Motor aus
      _delay_ms(50);
    } else if (entfernung >= 18) {
      shiftWrite(led[0]);
      OCR0B = 255;  // Motor an
      _delay_ms(50);
    } else {
      i = 18 - entfernung;
      shiftWrite(led[i]);
      OCR0B = ((8-i) * 255) / 8;
      _delay_ms(50);
    }


    entfernung = 0;
  }

  return 0;
}
