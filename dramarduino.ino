/*  _
** |_|___ ___
** | |_ -|_ -|
** |_|___|___|
**  iss(c)2020
**
**  public site: https://forum.defence-force.org/viewtopic.php?f=9&t=1699
**
**  Updated: 2020.10.05 - fixed bit operation - @john
**           2020.10.29 - fixed more bit operation - @thekorex
**  Modified: conditional FAST_RW turns on direct I/O for double the speed @retiredfeline
*/

/* ================================================================== */
#include <SoftwareSerial.h>

#define DI          A1  // PC1
#define DO           8  // PB0
#define CAS          9  // PB1
#define RAS         A3  // PC3
#define WE          A2  // PC2

#define XA0         A4  // PC4
#define XA1          2  // PD2
#define XA2         A5  // PC5
#define XA3          6  // PD6
#define XA4          5  // PD5
#define XA5          4  // PD4
#define XA6          7  // PD7
#define XA7          3  // PD3
#define XA8         A0  // PC0

#define M_TYPE      10  // PB2
#define R_LED       11  // PB3
#define G_LED       12  // PB4

#define RXD          0  // PD0
#define TXD          1  // PD1

#define BUS_SIZE     9

#define	FAST_RW

#ifndef	FAST_RW

#define	dR_DO()	digitalRead(DO)
#define	dWH(p)	digitalWrite((p),HIGH)
#define	dWL(p)	digitalWrite((p),LOW)

#else

#define	dR_DO()	(PINB & 1)

void dWH(int p) {
    switch (p) {
    case XA1:
        PORTD |= (1 << 2); break;
    case XA7:
        PORTD |= (1 << 3); break;
    case XA5:
        PORTD |= (1 << 4); break;
    case XA4:
        PORTD |= (1 << 5); break;
    case XA3:
        PORTD |= (1 << 6); break;
    case XA6:
        PORTD |= (1 << 7); break;
    case CAS:
        PORTB |= (1 << 1); break;
    case XA8:
        PORTC |= (1 << 0); break;
    case DI:
        PORTC |= (1 << 1); break;
    case WE:
        PORTC |= (1 << 2); break;
    case RAS:
        PORTC |= (1 << 3); break;
    case XA0:
        PORTC |= (1 << 4); break;
    case XA2:
        PORTC |= (1 << 5); break;
    default:
        Serial.println("Oops dWH"); break;
    }
}

void dWL(int p) {
    switch (p) {
    case XA1:
        PORTD &= ~(1 << 2); break;
    case XA7:
        PORTD &= ~(1 << 3); break;
    case XA5:
        PORTD &= ~(1 << 4); break;
    case XA4:
        PORTD &= ~(1 << 5); break;
    case XA3:
        PORTD &= ~(1 << 6); break;
    case XA6:
        PORTD &= ~(1 << 7); break;
    case CAS:
        PORTB &= ~(1 << 1); break;
    case XA8:
        PORTC &= ~(1 << 0); break;
    case DI:
        PORTC &= ~(1 << 1); break;
    case WE:
        PORTC &= ~(1 << 2); break;
    case RAS:
        PORTC &= ~(1 << 3); break;
    case XA0:
        PORTC &= ~(1 << 4); break;
    case XA2:
        PORTC &= ~(1 << 5); break;
    default:
        Serial.println("Oops dWL"); break;
    }
}

#endif

#undef  INLINE_OPS

#if defined FAST_RW && defined INLINE_OPS

#define	dWH_CAS	PORTB |= (1 << 1)
#define	dWL_CAS	PORTB &= ~(1 << 1)
#define	dWH_DI	PORTC |= (1 << 1)
#define	dWL_DI	PORTC &= ~(1 << 1)
#define	dWH_WE	PORTC |= (1 << 2)
#define	dWL_WE	PORTC &= ~(1 << 2)
#define	dWH_RAS	PORTC |= (1 << 3)
#define	dWL_RAS	PORTC &= ~(1 << 3)

#else

#define dWH_CAS	dWH(CAS)
#define dWL_CAS	dWL(CAS)
#define dWH_DI  dWH(DI)
#define dWL_DI  dWL(DI)
#define dWH_WE	dWH(WE)
#define dWL_WE	dWL(WE)
#define dWH_RAS	dWH(RAS)
#define dWL_RAS	dWL(RAS)

#endif

/* ================================================================== */
volatile int bus_size;

//SoftwareSerial USB(RXD, TXD);

const unsigned int a_bus[BUS_SIZE] = {
  XA0, XA1, XA2, XA3, XA4, XA5, XA6, XA7, XA8
};

void setBus(unsigned int a) {
  int i;
  for (i = 0; i < BUS_SIZE; i++) {
    a & 1 ? dWH(a_bus[i]) : dWL(a_bus[i]);
    a /= 2;
  }
}

void writeAddress(unsigned int r, unsigned int c, int v) {
  /* row */
  setBus(r);
  dWL_RAS;

  /* rw */
  dWL_WE;

  /* val */
  v & 1 ? dWH_DI : dWL_DI;

  /* col */
  setBus(c);
  dWL_CAS;

  dWH_WE;
  dWH_CAS;
  dWH_RAS;
}

int readAddress(unsigned int r, unsigned int c) {
  int ret = 0;

  /* row */
  setBus(r);
  dWL_RAS;

  /* col */
  setBus(c);
  dWL_CAS;

  /* get current value */
  ret = dR_DO();

  dWH_CAS;
  dWH_RAS;

  return ret;
}

void error(int r, int c)
{
  unsigned long a = ((unsigned long)c << bus_size) + r;
  digitalWrite(R_LED, LOW);
  digitalWrite(G_LED, HIGH);
  interrupts();
  Serial.print(" FAILED $");
  Serial.println(a, HEX);
  Serial.flush();
  while (1)
    ;
}

void ok(void)
{
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, LOW);
  interrupts();
  Serial.println(" OK!");
  Serial.flush();
  while (1)
    ;
}

void blink(void)
{
  digitalWrite(G_LED, LOW);
  digitalWrite(R_LED, LOW);
  delay(1000);
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, HIGH);
}

void green(int v) {
  digitalWrite(G_LED, v);
}

void fill(int v) {
  int r, c, g = 0;
  v &= 1;
  for (c = 0; c < (1<<bus_size); c++) {
    green(g? HIGH : LOW);
    for (r = 0; r < (1<<bus_size); r++) {
      writeAddress(r, c, v);
      if (v != readAddress(r, c))
        error(r, c);
    }
    g ^= 1;
  }
  blink();
}

void fillx(int v) {
  int r, c, g = 0;
  v &= 1;
  for (c = 0; c < (1<<bus_size); c++) {
    green(g? HIGH : LOW);
    for (r = 0; r < (1<<bus_size); r++) {
      writeAddress(r, c, v);
      if (v != readAddress(r, c))
        error(r, c);
      v ^= 1;
    }
    g ^= 1;
  }
  blink();
}

void setup() {
  int i;

  Serial.begin(9600);
  while (!Serial)
    ; /* wait */

  Serial.println();
  Serial.print("DRAM TESTER ");

  for (i = 0; i < BUS_SIZE; i++)
    pinMode(a_bus[i], OUTPUT);

  pinMode(CAS, OUTPUT);
  pinMode(RAS, OUTPUT);
  pinMode(WE, OUTPUT);
  pinMode(DI, OUTPUT);

  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);

  pinMode(M_TYPE, INPUT_PULLUP);
  pinMode(DO, INPUT);

  dWH_WE;
  dWH_RAS;
  dWH_CAS;

  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, HIGH);

  if (digitalRead(M_TYPE)) {
    /* jumper not set - 41256 */
    bus_size = BUS_SIZE;
    Serial.print("256Kx1 ");
  } else {
    /* jumper set - 4164 */
    bus_size = BUS_SIZE - 1;
    Serial.print("64Kx1 ");
  }
  Serial.flush();

  digitalWrite(R_LED, LOW);
  digitalWrite(G_LED, LOW);

  noInterrupts();
  for (i = 0; i < (1 << BUS_SIZE); i++) {
    dWL_RAS;
    dWH_RAS;
  }
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, HIGH);
}

void loop() {
  interrupts(); Serial.print("1"); Serial.flush(); noInterrupts(); fillx(0);
  interrupts(); Serial.print("2"); Serial.flush(); noInterrupts(); fillx(1);
  interrupts(); Serial.print("3"); Serial.flush(); noInterrupts(); fill(0);
  interrupts(); Serial.print("4"); Serial.flush(); noInterrupts(); fill(1);
  ok();
}
