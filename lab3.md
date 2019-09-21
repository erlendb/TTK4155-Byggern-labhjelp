### Lab 3

#### Oppgave 1

##### Verdt å lese

* Pinoversikten på side 2 i ADC-databladet

##### Verdt å vite

Dere har komponenten ADC0844 (20 bein), ikke AD0848 (24 bein).

VCC til 5V, DGND (digital ground) til jord.

CS (chip select) skal til riktig utgang på GAL (sannsynligvis pin 18 på GAL, alt etter hvordan dere programmerte GALen), men det kan lønne seg å koble CS rett til jord hvis dere ikke er sikre på om GAL funker som den skal. Det kan også lønne seg å koble begge CS på SRAM til jord sånn at SRAM garantert er skrudd av mens dere driver testing på ADC.

AGND (analog ground) og Vref (referansespenning) kan kobles rett til jord og 5V. Merk at AGND og Vref er innganger som sammenlignes med de analoge inngangene fra multifunksjonskortet når ADCen skal finne ut hvilken verdi som kommer fra joysticken og sliderne. Derfor kan det etterhvert være lurt å koble AGND og Vref på samme jord og samme spenning som dere har på multifunksjonskortet.

INTR-pinnen på ADC har med interrupts å gjøre. Det var en del av gruppene som begynte å tukle med interrupts på denne laben, men vi kjørte gjennom uten å bry oss om det.

WR og RD på ADC kobles på WR og RD på ATmega162 fordi ATmega162 skal skrive til og lese fra ADC.

CH1-CH4 er analoge innganger til ADC og skal etterhvert kobles til multifunksjonskortet.

DB0-DB7 på ADC er datautganger og skal kobles på de samme datainngangene på ATmega162 som du brukte til SRAM. DB0-DB3 (MA0-MA3) på ADC funker også som adresseinnganger. Fornuftig å koble DB0 på ADC til A0 på ATmega162 og DB7 til A7.

ADC har en innebygd latch. Så det som skjer når ATmega162 skal hente ut verdier fra ADC er følgende: ATmega162 sender ut en adresse til ADC. ADC lagrer adressen i latchen. ATmega162 begynner å lese fra ADC. ADC bruker adressen i latchen til å finne ut hva den skal sende til ATmega162.

#### Oppgave 2

##### Verdt å lese

* Kapittel 2.2 i P1000-userguide (databladet for multifunksjonskortet)

##### Verdt å vite

I oppgaven står det "Make sure the jumper across “EXTSEL” is connected". For å få OLED til å vise sliderposisjonen må EXTSEL-jumperen tas av. Det står altså feil i oppgaven.

Det er litt samme hvilken spenningskilde (USB eller spenningsgenerator) man bruker til multifunk-kortet. Vi valgte å koble til den samme spenningen som breadboardet får fra spenningsgeneratoren for å få felles jord.

#### Oppgave 3

##### Verdt å lese

* Kapittel 3.7 i P1000-userguide
* "Functional description", side 9 i ADC-databladet hvis dere vil forstå hvordan ADCen funker
* "TABLE 1. ADC0844 MUX ADDRESSING", side 9 i ADC-databladet
* "Timing diagrams", side 7 i ADC-databladet

##### Verdt å vite

Analoge verdier fra joysticken kommer ut på JOY-CONN på multifunk-kortet. Man kan ganske enkelt koble JOY-CONN pin 1 og pin 2 (se kp 3.7 i P1000-userguide) til CH1 og CH2 på ADC.

For å lese fra ADC må dere bruke uart, putty og `SRAM_init()`. Og så må dere skrive noe kode for å hente data fra ADC (lese data-koden ligner veldig på det som er i `SRAM_test()`).

ADC funker sånn at vi først skriver en verdi til ADC som forteller ADC hvilken kanal som skal leses. Deretter leser vi. I "Table 1", side 9 i ADC-databladet ser dere hvilken verdi dere må skrive til ADC for å lese den kanalen dere vil lese.

Mux mode differential tar spenningen på én kanal og trekker fra den andre. Mux mode single-ended leser bare verdien fra en kanal. Det er single-ended dere vil ha. I tabellen så ser man at for å lese CH1 single-ended, så må man skrive følgende til ADC: MA3:L, MA2:H, MA1:L, MA0:L - altså `0b0100` eller `0x04`. For å lese fra CH2 single-ended må man først skrive `0x05` til ADC. Og `0x06` for CH3 og `0x07` for CH4.

Les data-koden kan se ut som noe sånt som dette:

~~~c
#include <avr/io.h>
#define F_CPU 4915200 // For at util/delay.h ikke skal klage
#include <util/delay.h>
#include "uart.h"
#include "adc.h"

#define FOSC 4915200
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1


uart_init(MYUBRR);
adc_init(); // samme innhold som SRAM_init(). Kunne like gjerne brukt SRAM_init()

while(1) {
  volatile char *adc = (char *) 0x1400; // 0x1400 er start-adressen til ADC
  adc[0] = 0x04; // Skriver 0x04 til ADC, altså vi sier til ADC at vi skal hente ut info fra CH1

  // Her er det mulig dere må ha litt delay:
  // _delay_us(20);

  uint8_t value = adc[0]; // Leser 8-bit data fra ADC
  printf("ADC-verdi: %02X\n\n", value); //Printer til seriell aka putty

  _delay_ms(200); //Bare sånn at det ikke blir helt kaos i putty
}

~~~

Sjekk "Timing diagrams" i ADC-databladet for å forstå hvordan ting funker i ADCen. WR- og RD-signal fikser ATmega162 på egenhånd. Legg merke til aktiv lav CS. Hvis dere vil bruke interrupts så ser dere at INTR-utgangen på ADC blir høy etter at ADC har mottatt adressen fra ATmega162. Da kan INTR gi signal til ATmega162 om at ATmega162 kan begynne å lese data fra ADC.

#### Oppgave 4

##### Verdt å vite

Pin 1 (?) (helt til venstre) på PORTB og pin 5 (?) (midten) på PORTD. Sjekk med oscilloskopet for sikkerhets skyld :).

#### Oppgave 5

##### Verdt å lese

* Kapittel 3.8 i P1000-userguide

##### Verdt å vite

Sliderutgangene på PORTB og PORTD skal bare rett inn på FILTER-inngangene. FILTER-utgangene kobles på CH3 og CH4 på ADC.

#### Oppgave 6

##### Verdt å vite

Touchknapp-pinnene er rett ved slider-pinnene på PORTB og PORTD. Husk også joystick-knappen på JOY-CONN pin 3. Vi koblet de tre knappene inn på PB0-PB2 på ATmega162.

#### Oppgave 7-8

Se en eller annen kok.
