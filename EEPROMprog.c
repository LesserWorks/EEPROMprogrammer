#include <avr/io.h>
#include <util/delay_basic.h>
#include <stdint.h>
#define SHIFT_DATA 2
#define SHIFT_CLK 3
#define SHIFT_LATCH 4
#define EEPROM_D0 5
#define EEPROM_D7 12
#define WRITE_EN 13
#define WE_high() PORTD |= 1 << 5;
#define WE_low() PORTD &= ~(1 << 5);
#define OE_high() PORTD |= 1 << 7;
#define OE_low() PORTD &= ~(1 << 7);
/*
Using Arduino Nano
Reg 1 has A0-A7 in QA-QH
Reg 2 has A8-A14 on QA-QG, then OE on QH
They are cascaded, reg 1 --> reg 2.
Serial in is 2, clock is 3, latch is 4. WE goes to 13, CE to GND
EEPROM to Nano connections:
D0 -> 5
D1 -> 6
D2 -> 7
D3 -> 8
D4 -> 9
D5 -> 10
D6 -> 11
D7 -> 12
*/

// Pins required: 15 address lines, 8 data lines, WE, OE
// Use PORTC for data, PORTB for A0-A7, PORTA<0..6> for A8-A14
// PORTD5 will be WE
// PORTD7 will be OE
// 32,768 bytes in AT28C256
#if 0
void setAddress(int address, bool outputEnable) {
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, (address >> 8) | (outputEnable ? 0x00 : 0x80)); // 0b10000000 MSB set
  // This puts A8-A14 and OE state on reg 2
  // If outputEnable is true, OE is low, high otherwise
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address);
  // This puts A0-A7 on reg 1.

  digitalWrite(SHIFT_LATCH, LOW);
  digitalWrite(SHIFT_LATCH, HIGH);
  digitalWrite(SHIFT_LATCH, LOW);
}
#endif
// Keep CE always low
// During read, OE must be low to allow data to be output, and WE must be high
// During write, OE is high, WE pulses low, then high. WE must stay low for at least 100ns
// To page write, put address and data on bus, pulse WE, put new address and data, pulse WE etc.
// A6-A14 must be constant during page write, A0-A5 can change to specifiy any 64 byte region.
// 10ms to complete page write
// To erase entire chip, AA to 5555, 55 to 2AAA, 80 to 5555, AA to 5555, 55 to 2AAA, 10 to 5555, then wait 20ms
// Just like a page write
// While write is in progress, you can put WE and OE high, and keep last used address on bus. Then toggle OE low and high,
// During each toggle, IO pin 7 will output the opposite of what was last written to it until write is finished, 
// then it will output actual bit like a regular read. Also, IO pin 6 will keep toggling until write is finished,
// then it stops toggling.
// During successive reads or writes, no need to toggle OE
/*
 * Read a byte from the EEPROM at the specified address.
 */
#if 0
byte readEEPROM(int address) {
  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    pinMode(pin, INPUT);
  }
  setAddress(address, /*outputEnable*/ true); // OE low

  byte data = 0;
  for (int pin = EEPROM_D7; pin >= EEPROM_D0; pin -= 1) {
    data = (data << 1) + digitalRead(pin);
  }
  return data;
}
#endif
void eeSetup(void)
{
  DDRA = 255; // Address bus is output
  DDRB = 255;
  DDRC = 0; // Data bus starts as input
  DDRD |= (1 << 5) | (1 << 7); // OE and WE are outputs
  OE_low();
  WE_high();
}
uint8_t eeReadByte(const uint16_t addr)
{
  PORTA = addr >> 8; // High
  PORTB = addr & 255; // Low
  _delay_loop_2(2); // Delays at least 8 CPU cycles to ensure at least 350ns pass even with 20MHz clock
  return PORTC; // Read data bus
}
void eeWriteByte(const uint16_t addr, const uint8_t data)
{
  DDRC = 255; // Data bus is output
  OE_high();
  PORTA = addr >> 8;
  PORTB = addr & 255;
  PORTC = data;
  WE_low();
  WE_low(); // Has effect of delaying for 2 clock cycles so WE stays low for at least 100ns
  WE_high();
  DDRC = 0; // Data bus is input to enable polling
  OE_low();
  OE_low(); // Delay 2 cycles
  do
  {
    prev = PINC & (1 << 6);
    OE_high();
    _delay_loop_1(1); // Delay 3 cycles
    OE_low();
    OE_low(); // Delays for 2 clock cycles to IO6 is not read for at least 100ns
  }
  while(prev ^ (PINC & (1 << 6))); // True when PINC<6> and prev<6> have different values
  return;
}
  



/*
 * Write a byte to the EEPROM at the specified address.
 */
#if 0
void writeEEPROM(int address, byte data) {
  setAddress(address, /*outputEnable*/ false); // OE high
  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    pinMode(pin, OUTPUT);
  }

  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    digitalWrite(pin, data & 1);
    data = data >> 1;
  }
  digitalWrite(WRITE_EN, LOW);
  delayMicroseconds(1);
  digitalWrite(WRITE_EN, HIGH);
  delay(10);
}
#endif

/*
 * Read the contents of the EEPROM and print them to the serial monitor.
 */
void printContents() {
  for (int base = 0; base <= 255; base += 16) {
    byte data[16];
    for (int offset = 0; offset <= 15; offset += 1) {
      data[offset] = readEEPROM(base + offset);
    }

    char buf[80];
    sprintf(buf, "%03x:  %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x",
            base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

    Serial.println(buf);
  }
}


// 4-bit hex decoder for common anode 7-segment display
byte data[] = { 0x81, 0xcf, 0x92, 0x86, 0xcc, 0xa4, 0xa0, 0x8f, 0x80, 0x84, 0x88, 0xe0, 0xb1, 0xc2, 0xb0, 0xb8 };

// 4-bit hex decoder for common cathode 7-segment display
// byte data[] = { 0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b, 0x77, 0x1f, 0x4e, 0x3d, 0x4f, 0x47 };


void setup() {
  // put your setup code here, to run once:
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  digitalWrite(WRITE_EN, HIGH);
  pinMode(WRITE_EN, OUTPUT);
  Serial.begin(57600);

  // Erase entire EEPROM
  Serial.print("Erasing EEPROM");
  for (int address = 0; address <= 2047; address += 1) {
    writeEEPROM(address, 0xff);

    if (address % 64 == 0) {
      Serial.print(".");
    }
  }
  Serial.println(" done");


  // Program data bytes
  Serial.print("Programming EEPROM");
  for (int address = 0; address < sizeof(data); address += 1) {
    writeEEPROM(address, data[address]);

    if (address % 64 == 0) {
      Serial.print(".");
    }
  }
  Serial.println(" done");


  // Read and print out the contents of the EERPROM
  Serial.println("Reading EEPROM");
  printContents();
}


void loop() {
  // put your main code here, to run repeatedly:

}
