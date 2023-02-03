// Start Setup

// Start Includes
#include "lcd.h"
#include <avr/io.h>
#include <util/delay.h>
// End Includes

// Clock rate
#define F_CPU 16000000ul;

// Start Shift register Setup
#define HC595_PORT PORTB
#define HC595_DDR DDRB
#define HC595_DS_POS PB0     //Data pin (DS) pin location
#define HC595_SH_CP_POS PB1  //Shift Clock (SH_CP) pin location
#define HC595_ST_CP_POS PB2  //Store Clock (ST_CP) pin location

// Change data (DS)lines
#define HC595DataHigh() (HC595_PORT |= (1 << HC595_DS_POS))
#define HC595DataLow() (HC595_PORT &= (~(1 << HC595_DS_POS)))

// Start Shift register functions 
// Initialise Shift register
void shiftInit() {
  //Make the Data(DS), Shift clock (SH_CP), Store Clock (ST_CP) lines output
  HC595_DDR |= ((1 << HC595_SH_CP_POS) | (1 << HC595_ST_CP_POS) | (1 << HC595_DS_POS));
}

// Sends a clock pulse on SH_CP line
void shiftPulse() {
  // Pulse the Shift Clock
  HC595_PORT |= (1 << HC595_SH_CP_POS);     //HIGH
  HC595_PORT &= (~(1 << HC595_SH_CP_POS));  //LOW
}

// Sends a clock pulse on ST_CP line
void shiftLatch() {
  // Pulse the Store Clock
  HC595_PORT |= (1 << HC595_ST_CP_POS);  //HIGH
  _delay_loop_1(1);
  HC595_PORT &= (~(1 << HC595_ST_CP_POS));  //LOW
  _delay_loop_1(1);
}


void shiftWrite(uint8_t data) {
  // Send each 8 bits serially
  // Order is MSB first
  for (uint8_t i = 0; i < 8; i++) {
    // Output the data on DS line according to the
    // Value of MSB
    if (data & 0b10000000) {
      // MSB is 1 so output high
      HC595DataHigh();
    } else {
      // MSB is 0 so output high
      HC595DataLow();
    }
    shiftPulse();      // Pulse the Clock line
    data = data << 1;  // Now bring next bit at MSB position
    _delay_ms(5);
  }
  // Now all 8 bits have been transferred to shift register
  // Move them to output latch at one
  shiftLatch();
}
// End Shift register funktions
// End Shift register Setup

// variable Setup
long entfernung;
// End Setup

// Start Main
int main(void) {

  // Initialise Display
  lcd_init(LCD_DISP_ON);

  // Array used for Bar-Chart
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
  
  // Variables for Bar-Chart
  uint8_t i;
  uint8_t j;
  
  // Variable for Distance as String
  char int_str[20];
  
  // Initialise Shift register
  shiftInit();

  // Set Data Direktion register
  DDRD = 0xff; // Ultra Sonic Sensor
  DDRD &= ~(1 << PD3); // Ultra Sonic Sensor Echo as Input
  DDRH = 0xff; // LED
  DDRA = 0xff; // Motor 
  DDRG |= (1 << DDD5); // Motor controller PWM
 
  // Setup PWM for Motor controll
  TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0B1);
  TCCR0B |= (1 << CS01);

  // Set Pull-up for Ultra Sonic Echo 
  PORTD |= (1 << PD3);

  // Start Main Loop
  while (1) {

    // Send Ultra Sonic Pulse of 10ms
    PORTD |= (1 << PD2); // Start sending
    _delay_ms(10); //wait 10ms
    PORTD &= ~(1 << PD2); // Stop sending

    // wait for echo
    while (!(PIND & (1 << PD3))) {/* wait */ }

    // Messure Distance using Echo
    while (PIND & (1 << PD3)) {
      entfernung = entfernung + 1;
      _delay_us(58);
    }

    // Display Distance on Display
    // Write static text
    lcd_gotoxy(0,2);
    lcd_puts_p(PSTR("Entfernung in cm: "));
    lcd_gotoxy(30,2);
    // Conditional clearing
    if(entfernung<100){
      lcd_gotoxy(33,2);
      lcd_puts(" ");
    }
    if(entfernung<10){
      lcd_gotoxy(32,2);
      lcd_puts(" ");
    }

    // String of Distance
    itoa(entfernung,int_str,10);
    
    // Write Distance
    lcd_puts(int_str);
    
    // Controll Motor using PWM
    if (entfernung <= 10) { // Stop
      shiftWrite(led[8]);
      OCR0B = 0;
      _delay_ms(50);
    } else if (entfernung >= 18) { // Full Speed
      shiftWrite(led[0]);
      OCR0B = 255;  
      _delay_ms(50);
    } else { // Dynamic Speed
      i = 18 - entfernung;
      shiftWrite(led[i]);
      OCR0B = ((8-i) * 255) / 8;
      _delay_ms(50);
    }

    // Reset Distance
    entfernung = 0;
  }
  // End Main loop

  return 0;
}
//End Main
