### Lab 2

Hvis du ikke er helt stødig på bitoperasjoner i C, så kan det være en fordel å lese litt på det. De viktigste greiene er kanskje disse:

~~~
// Setter bit uten å bry oss om hva som var lagret fra før
uint8_t byte = 0b1010 1010;
byte = (1 << 2); // gir byte: 0b0000 0100
byte = (1 << 2)|(1 << 4); // gir byte: 0b0001 0100

// Setter bit uten å påvirke resten av byten
uint8_t byte = 0b1010 1010;
byte |= (1 << 2); // gir byte: 0b1010 1110
byte |= (1 << 2)|(1 << 4); // gir byte: 0b1011 1110

// Sjekker en bit på gitt plass i byten
uint8_t byte = 0b1010 1010;
bit = (byte & (1 << 2)); // gir bit: 0
bit = (byte & (1 << 3)); // gir bit: 1
~~~

#### L2 Oppgave 1

##### Kjekt å lese

* Les "description", side 1 i latch-databladet for en recap på hvordan vipper funker.
* "External memory interface", side 26 i ATmega162-databladet. Verdt å lese hele kapitlet (til side 30). Det er ikke farlig om du ikke skjønner timingdiagrammene.

##### Kjekt å vite

(Ikke tenk på GAL-greiene før oppgave 3.)

Et forsøk på å forklare vippekaoset: Greia her er at en adresse (i vårt tilfelle 12 bit) og litt data (i vårt tilfelle 8 bit) skal havne nede i SRAM samtidig. Men Atmega162 kan kun sende 12 bit om gangen (iallfall når JTAG er koblet til, som det er i vårt tilfelle). Så derfor må vi dele det litt opp og bruke en vippe (latch). Først sender vi 8 bit av adressen til vippa. Vippen "lagrer" dette. Deretter sender vi ut data ut fra Atmega162 på de samme pinnene som vi sendte adresse ut på i stad. Men nå er vippen koblet fra, så nå har vi altså adressebits ut fra vippa og data ut fra Atmega162. I tillegg sender vi de siste fire adressebitene ut fra Atmega162 på noen helt andre pinner. Og så tar vi alt sammen og stapper det inn i SRAM.

Åkei, vippa (latchen) skal på plass. VCC og GND må kobles til. Husk decoupling-kondensator. ALE på Atmega162 skal kobles på LE på vippa (ALE=Address latch enable, LE=Latch enable). OE (Output enable) må kobles til jord (fordi OE er en invertert inngang). Og så skal PA0-PA7 på Atmega162 inn på 1D-8D på vippa. Dette er altså de åtte adressebitene som skal "lagres" i vippa.

Nå kan det lønne seg å teste om vippa funker. Test med lysdioder eller oscillator på "utsiden" av vippa (den siden av vippa som peker bort fra Atmega162).

Et testprogram kan for eksempel se ut som dette:

~~~c
#include <avr/delay.h>
// mulig avr/delay.h ikke funker, og du må bruke følgende:
// #define F_CPU 4915200
// #include <util/delay.h>

int main() {
  DDRE = 0b10; // Setter ALE (pin 1 på port E) som utgang. 0=inngang, 1=utgang

  PORTE = 0b10; // Setter ALE høy. Forteller vippa at nå kommer det en adresse som skal lagres.
  PORTA = 0b000001; // = 0x00. Sender en adresse ut til vippa.

  _delay_ms(2000); // 2 sek delay for å gjøre det enklere å måle på kretsen.

  PORTE = 0b00; // Setter ALE lav. Nå lagres adresseverdien i vippa.

  _delay_ms(2000);

  PORTA = 0b01010101; // Sender ut ny adresse. Ingenting skjer på baksiden av vippa.

  _delay_ms(2000);

  PORTE = 0b10; // Gammel adresse fjernes fra vippa og den nye sendes gjennom.

  //...

}
~~~

#### L2 Oppgave 2

##### Kjekt å lese

* "External memory interface", side 26 i ATmega162-databladet gjelder her også.
* "Truth table(1,2,3)", side 2 i SRAM-databladet.
* "Special Function IO Register – SFIOR", side 32 i ATmega162-databladet. Forklarer maskering av pinner. Se spesielt "Table 4".
* "MCU Control Register – MCUCR", side 30 i ATmega162-databladet. Det viktige her er egentlig setningen "Writing SRE to one enables the External Memory Interface".


##### Kjekt å vite

SRAM. Koble opp GND, VCC og kondensator. NC (Not connected) skal ikke kobles til noe.

CS2 skal ha 5V. CS1 skal til jord. (Dette er foreløpige greier. CS1 og CS2 skal brukes til noe fornuftig i oppgave 3.) Sjekk Truth table, side 2 i SRAM-databladet, så ser du at de må kobles sånn for å aktivere skriving og lesing til SRAM.

RD (read data strobe) på ATmega162 skal kobles til OE (Output enable) på SRAM.
WR (write data strobe) på ATmega162 skal kobles til WE (Write enable) på SRAM.

SRAM skal ha adresse og data. Adressen som SRAM skal ha kommer ut fra vippa (8 bit) + fra ATmega162 (pinnene PC0-PC3) (4 bit). PC4-PC7 skal ikke brukes, for de er opptatt med JTAG. Du har altså til sammen 12 adresseledninger som skal inn på SRAM. På SRAM har du 13 adresseinnganger (A0-A12). Det har ikke noe å si hvilken adressebit fra ATmega162/vippa som går inn på hvilken adresseinngang på SRAM. Det har ikke noe å si hvilken adresseinngang på SRAM som du ikke bruker. Ingenting har noe å si. Bare koble så det ser pent ut. Den adresseinngangen på SRAM som du ikke bruker, kobler du til jord (adressebiten blir altså satt til 0).

Hvorfor har det ikke noe å si hvilken adressebit som kobles på hvilken inngang? Fordi du leser og skriver til SRAM fra de samme pinnene på ATmega162. Det blir litt som om du skal legge noe i hylle nr. 2, men du legger det i hylle nr. 4 istedenfor (fordi du tror at hylle nr. 4 er hylle nr. 2). Når du skal hente det ut fra hylla igjen, så vet du at det er i hylle nr. 2 - men fordi du tror at hylle nr. 4 er hylle nr. 2 så henter du det fra hylle nr. 4 - og det var jo der du la det. Lenge leve dårlige analogier.

Det som står igjen nå er koble data fra ATmega162 til SRAM. Data kommer ut på PA0-PA7 på ATmega162 (ja, de samme pinnene som du har koblet til vippa) og skal kobles til I/O0-I/O7 på SRAM. Her kan det være greit å koble ting i riktig rekkefølge, så du slipper å få krøll på dataen din.

Nå kan du teste SRAM-en. Vi hardkokte eksempeltestprogrammet som vi fikk utdelt i oppgaven. I tillegg må du bruke uart-driveren som du lagre i lab1, og så må du huske å initiere SRAM-en:

*main.c*:
~~~c
#include <avr/io.h>
#include "uart.h"
#include "sram.h"

#define FOSC 4915200 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1 // 31


void main(void) {
  uart_init(MYUBRR); //Ikke noe nytt her. Samme som i lab1
  SRAM_init(); //Kokt fra labforelesning
  SRAM_test(); //Kokt fra oppgaveteksten
}
~~~

*sram.c*:
~~~c
#include "sram.h"

void SRAM_init() {
  MCUCR = (1 << SRE); // Enable SRAM i ATmega162
  SFIOR = (1 << XMM2); // Maskerer PC4-PC7 på ATmega162. Dvs passer på at vi ikke kuker med JTAG-pinnene. Sjekk "Table 4", side 32 i ATmega162-databladet.
}

void SRAM_test(void) {
  volatile char *ext_ram = (char *) 0x1800; // Start address for the SRAM
  uint16_t ext_ram_size = 0x800;
  uint16_t write_errors = 0;
  uint16_t retrieval_errors = 0;
  printf("Starting SRAM test...\n");
  // rand() stores some internal state, so calling this function in a loop will
  // yield different seeds each time (unless srand() is called before this function)
  uint16_t seed = rand();
  // Write phase: Immediately check that the correct value was stored
  srand(seed);
  for (uint16_t i = 0; i < ext_ram_size; i++) {
    uint8_t some_value = rand();
    ext_ram[i] = some_value;
    uint8_t retreived_value = ext_ram[i];
    if (retreived_value != some_value) {
      printf("Write phase error: ext_ram[%4d] = %02X (should be %02X)\n", i, retreived_value, some_value);
      write_errors++;
    }
  }
  // Retrieval phase: Check that no values were changed during or after the write phase
  srand(seed); // reset the PRNG to the state it had before the write phase
  for (uint16_t i = 0; i < ext_ram_size; i++) {
    uint8_t some_value = rand();
    uint8_t retreived_value = ext_ram[i];
    if (retreived_value != some_value) {
      printf("Retrieval phase error: ext_ram[%4d] = %02X (should be %02X)\n", i, retreived_value, some_value);
      retrieval_errors++;
    }
  }
  printf("SRAM test completed with \n%4d errors in write phase and \n%4d errors in retrieval phase\n\n", write_errors, retrieval_errors);
}
~~~

*sram.h*:
~~~c
#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>

void SRAM_init();
void SRAM_test(void);
~~~

`SRAM_test()` gir tilbakemeldinger fra testen gjennom uart, altså må du koble seriellkabelen til PCen og åpne putty for å se hva testen sier.

`SRAM_test()` lagrer verdier på alle adressene i SRAM, og etterpå leser den av de samme adressene igjen. Hvis den leser av det samme som den har lagres, så får du beskjed om at alt er på stell. Hvis den leser av noe annet enn det den lagres, så får du beskjed om at det er noe feil her.

Hvis du ikke får opp noe som helst i putty så har ikke putty kontakt med kretsen. Hvis testprogrammet sier at det ikke er noen feil, så har du gjort alt riktig. Hvis du får en feil eller to, så har du antakelig gjort alt riktig - feil kan forekomme iblant likevel. Hvis du får masse feil så har du antakelig koblet noe feil. Hvis testprogrammet leser av feil verdi på alle adresseplassene (og gjerne akkurat samme verdi hver gang), så er det sannsynligvis noe feil med initieringen av SRAM (`SRAM_init()`).

(Du trenger ikke å definere ALE som utgang (`DDRE = 0b10`) i SRAM-testprogrammet.)

#### L2 Oppgave 3

##### Kjekt å lese

* De første tre kapitlene av "VHDL tutorial" på https://www.seas.upenn.edu/~ese171/vhdl/vhdl_primer.html
* Pinoversikten på første side av GAL-databladet

##### Kjekt å vite

Når du sender data til en adresse fra ATmega162 så ser det omtrent ut som dette: `memory[0x1A75] = 1;`. Den kodelinja der lagrer `1` på adressen `0x1A75`. Det betyr at `1` lagres på den adressen i SRAM-minnet. Men hvis du skriver `memory[0x13FF] = 1;` så sender du `1` til OLED. Dette skal ikke lagres på SRAM, selv om det kan se litt sånn ut.

Poenget med GAL er å finne ut hvilken komponent som ATmega162 prøver å sende data til. Først må vi definere adresseområdene. Denne forklaringen tar utgangspunkt i adresseområdene beskrevet i "Figure 13" i oppgaveteksten, altså følgende:

~~~
Dings                  Adresse
                    Hex       Binær
SRAM slutt:         0x1FFF    1 111 111111111
SRAM start:         0x1800    1 100 000000000
ADC slutt:          0x17FF    1 011 111111111
ADC start:          0x1400    1 010 000000000
OLED-data slutt:    0x13FF    1 001 111111111
OLED-data start:    0x1200    1 001 000000000
OLED-command slutt: 0x11FF    1 000 111111111
OLED-command start: 0x1000    1 000 000000000
~~~

GAL skal altså passe på at når vi sender noe til adresser mellom `0x1800` og `0x1FFF` så går det til SRAM, når vi sender noe til `0x1400`-`0x17FF` så går det til ADC osv. Dette gjøres ved å koble adressebitene inn på GAL-en, og ved å sjekke hvilke av pinnene som er høye/lave, så finner vi ut hvilken komponent som skal ta imot data. Alle komponentene som skal ta imot data har "skru på"-innganger (chip select aka CS), så GAL-en skrur rett og slett på den komponenten som skal ha data, og så skrur den av de andre.

For å progge GAL-en må du finne ut hva som skiller de ulike adressene. Binærtallene over er det som faktisk kommer ut av pinnene på ATmega162 når den sender ut adressen. Pinnen/binærsifferet helt til venstre er `1` hele tiden, så vi kan ikke skille adresser ved hjelp av dette. De tre neste bitene, derimot, kan vi bruke til å skille adressene fra hverandre. Hver gang noe sendes til OLED-command, så vil de tre bitene være `000` (se tabellen). Hver gang vi sender noe til OLED-data, så vil de tre bitene være `001`. Hver gang vi sender noe til ADC, så vil de tre bitene være `01x` (x: tilfeldig). De tre bitene av SRAM-adressene er alltid `1xx`. Med disse "reglene" kan vi skille mellom adressene til de forskjellige komponentene.

Nå som vi har funnet de tre bitene vi vil bruke til å finne riktig komponent, så må de tre bitene kobles inn på GAL-en. Det mest signifikante bitet av de tre bitene vi skal bruke (altså bitet NEST lengst til venstre i binærtallene over) sendes ut fra ATmega162 på PC3 (aka A11, som er det høyeste tallet vi bruker av adressepinnene). Neste bit (aka binærsifferet til høyre for det forrige) sendes ut på PC2 (A10). Det tredje på PC1 (A9). Med andre ord: vi vil koble PC3-PC1 (A11-A9) fra ATmega162 inn på GAL-en.

Pin 1-10 på GAL-en er innganger. Pin 12-19 er utganger (kanskje også pin 11, hvem vet). Så du vil koble de tre pinnene fra ATmega162 inn på for eksempel pin 1, 2 og 3 på GAL.

Og så må jo GAL-en progges, da. Tar utganspunkt i følgende nesten ferdige program fra oppgaveteksten:

~~~VHDL
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity address_decoder is
	Port (
		a11 : In std_logic;
		a10 : In std_logic;
		a9  : In std_logic;

		ram_cs  : Out std_logic;
		adc_cs  : Out std_logic;
		oled_cs : Out std_logic;
		oled_dc : Out std_logic
	);
	attribute LOC : string;
	attribute LOC of ram_cs  : signal is "P19";
	attribute LOC of adc_cs  : signal is "P18";
	attribute LOC of oled_cs : signal is "P17";
	attribute LOC of oled_dc : signal is "P16";

	attribute LOC of a11 : signal is "P1";
	attribute LOC of a10 : signal is "P2";
	attribute LOC of a9  : signal is "P3";
end address_decoder;

architecture behave of address_decoder is begin

  -- din superkode her

end behave;

~~~

I koden over definerer vi a11, a10 og a9. Litt lenger ned i koden setter vi dem til henholdsvis pin1, pin2 og pin3. Tilsvarende gjøres med ram_cs, adc_cs, oled_cs og oled_dc. Tanken her er altså at hvis det kommer en adresse inn som GAL skjønner at hører til SRAM, så skal SRAM-utgangen (pin 19) på GAL settes høy, mens alle de andre pinnene settes lave. Tilsvarende for ADC og OLED.

Istedenfor "din superkode her" i koden over kan du putte inn noe sånt som dette:

~~~vhdl
  ram_cs <= a11;
~~~

Det betyr rett og slett at ram_cs (altså pin 19) settes til det samme som a11 (pin 1). Forhåpentligvis har du koblet A11 fra ATmega162 inn på pin 1 på GAL. Isåfall vil altså pin19-utgangen på GAL settes høy når addressebitet fra A11 fra ATmega162 er høyt. Og et par avsnitt over her ble vi jo enige om at det er nettopp dét bitet som skiller SRAM-adressene fra de andre adressene.

I forrige oppgave satte vi CS1- og CS2-inngangene på SRAM til jord og 5V midlertidig. Det er nettopp disse to inngangene som er "skru på"-inngangene til SRAM (sjekk "Truth table(1,2,3)" på andre side i SRAM-databladet). Hvis du vil at GAL-en skal skru av og på SRAM, så kan det lønne seg å koble CS2 til utgangen (pin19) på GAL. CS1 kan fortsatt kobles til jord.

Tilsvarende logikk gjør at vi ender opp med følgende for de andre utgangene på GAL:

~~~vhdl
  ram_cs <= a11;
  adc_cs <= NOT (NOT a11 AND a10);
  oled_cs <= NOT (NOT a11 AND NOT a10);
  oled_dc <= NOT a11 AND NOT a10 AND a9;
~~~

Merk at noen komponenter krever at chip select (CS) settes høy når komponenten skal brukes, mens andre komponenter krever at CS settes lav når komponenten skal brukes. SRAM skal ha høy CS når den brukes. ADC skal ha lav CS når den er i bruk. Derfor er det en ekstra `NOT()` i verdien som settes til `adc_cs`. Hvis du sammenligner med tabellen over så ser du at `adc_cs <= NOT a11 AND a10;` ville gjort ADC-utgangen på GAL høy når vi skal aktivere ADC. Mens `adc_cs <= NOT (NOT a11 AND a10);` altså inverterer dette, sånn at vi får lavt signal ut når ADC skal aktiveres.

Følg oppskriften i oppgaveteksten for å progge GAL-en. Mulig du må trykke på "Show obsolete devices" i ispLever hvis du ikke finner "GAL Device" under "Device family". Kjør `SRAM_test()`-programmet fra oppgave 2 når GAL-en er ferdigprogget og CS2 på SRAM er koblet til GAL-en.
