# Byggmester Bob

> Man bruker 15 timer hver uke på lab. Minst!
> &mdash; *Student (23)*

> Velg romtek
> &mdash; *Student (22)*

> Ingen skjønner noenting. Ikke studassene heller
> &mdash; *Emneansvarlig (47)*

> Sorry gutta, jeg aner ikke hvorfor det ikke funker
> &mdash; *Studass (24)*

> Just do it as described in the datasheet
> &mdash; *Oppgaveteksten*

\
Dette er den byggernhjelpen jeg skulle ønske jeg hadde hatt selv. Lykke til :)

Noe feil i denne greia som bør fikses? Send meg ei melding, eller legg inn en issue eller en pull request.

## Labhjelp

Hele labhjelpen finner du [på forsiden](https://github.com/erlendb/TTK4155-Byggern-labhjelp). \
Lab 1 finnes i [lab1.md](lab1.md). \
Lab 2 finnes i [lab2.md](lab2.md). \
Lab 3 finnes i [lab3.md](lab3.md). \
Lab 4 finnes i [lab4.md](lab4.md).

**Supertips 1**: Hvis dere er ganske sikre på at dere har gjort alt riktig, og koken har gjort det samme som dere (og studass ikke skjønner hva som er feil), så kan det godt hende at en av komponentene deres ikke funker. Lån naboen sin komponent og test med den, så slipper dere å bruke evig mye tid før dere skjønner at det er komponenten som er skyldig.

**Supertips 2**: Når dere har koblet opp noe og testet at det funker, så klipper dere kanskje til noen nye ledninger og kobler om sånn at brødbrettet ser pent ut. Her er det fort gjort å gjøre feil. Og hvis dere kobler én ledning én plass feil når dere kobler om, så risikerer dere å bruke et par timer på å finne feilen. Så gjør det riktig :).

### Lab 1

#### L1 Oppgave 1

##### Kjekt å lese

* Figurene på første side av LM7805-databladet

##### Kjekt å vite

Ta utgangspunkt i "Figure 3 - breadboard" fra oppgaveteksten. Det har ikke noe å si om dere har fire rader på breadboardet istedenfor tre eller om spenningskontaktene på siden står motsatt vei.

Det kan lønne seg å begynne med å plassere alt sånn som det er vist i "Figure 5" fra oppgaveteksten. Da slipper dere å rive opp alt dere har koblet senere fordi dere må få plass til en ny komponent :).

Spenningsregulatoren LM7805 skal kobles opp sånn som det står til høyre på første side i LM7805-databladet. Vi brukte 100 µF (?) på kondensatoren på output.

Hvis dere kobler høy spenning (10-12 V) rett inn på komponentene deres, så risikerer dere å ødelegge komponentene. Om dere gjør det er det bare å hente ny komponent, men det er like greit å bare ikke drepe komponentene. Så jeg anbefaler alle å ha forsyningsspenning (10-12V) på én av spenningslinjene på breadboardet, og så sette opp 5V på alle de andre spenningslinjene med én gang. (Vi klarte å koble SRAM-brikken til 10V og brukte litt tid på å finne ut om den fortsatt funket.)

#### L1 Oppgave 2

##### Kjekt å lese

* Om decoupling capacitors i oppgaveteksten. Andre avsnitt under "Decoupling", side 10.
* "Figure 1", side 1 i ATmega162-databladet. Oversikt over pinnene på ATmega162.

##### Kjekt å vite

5V fra spenningsregulatoren skal inn på VCC på ATmega162. GND til jord.

"Decoupling capacitor" kan være 100 µF (?). Kobles mellom pin 1 (VCC) på ATmega162 og jord.

#### L1 Oppgave 3

##### Kjekt å lese

* Kapittel 2 ("Connection of reset pins on AVR") i AVR-userguide. Les hele siden, inkludert underkapittel 2.2
* Les om "RESET" under "Pin descriptions", side 6 i ATmega162-databladet.

##### Kjekt å vite

Sjekk side 5-6 i AVR-userguide. Kort fortalt skal dere koble opp kretsene som er i figurene på side 6.

Punktet der det står "RESET" i "Figure 2-3" skal kobles på der det står "Reset" i "Figure 2-2". "SW" er en trykknapp. Husk å måle på knappen hvilken vei den kortsluttes når man trykker på den. Ikke tenk på de rare linjene på høyre og venstre side av "SW" i "Figure 2-3".

Reset på ATmega162 er pin 9.

#### L1 Oppgave 4

##### Kjekt å lese

* Søk på "crystal" i AVR-userguide. Ikke så mye interessant her, men kan hjelpe på forståelsen av hva en krystall er.
* Les under "Crystal Oscillator", side 36 i ATmega162-databladet. "Figure 19" viser koblingsoppsettet.

##### Kjekt å vite

Krystallen dere skal bruke er en av de to små sølvfargede rare greiene. De har ulike frekvenser, og frekvensen på krystallen står på toppen av den. Den som det står 4.9152 på skal kobles mellom pin 19 (XTAL1) og pin 18 (XTAL2) på ATmega162. Og så kondensatorer fra hvert bein på krystallen til jord.

#### L1 Oppgave 5

##### Kjekt å lese

* "Alternate functions of port C", side 75 i ATmega162-databladet. Legg merke til JTAG-pinnene i "Table 35".
* Kapittel 3.2 "Connecting to a JTAG target", side 13 i Atmel-ICE-userguide.
* (Kapittel 4.2.1 "JTAG", side 19 i Atmel-ICE-userguide. (Alt nedenfor "Table 4-1" er IKKE relevant.) Ikke sikkert du trenger dette kapitlet i det hele tatt.)

##### Kjekt å vite

Atmel-ICE er den hvite dingsen som minner litt om en powerbank. Når denne kobles til breadboardet skal dere bruke "AVR"-utgangen på Atmel-ICE (altså ikke "SWD"). Det er fordi ATmega162 er en AVR-mikroprosessor.

Atmel-ICE kan kobles til breadboardet på to måter som er helt like:
* Via den lille overgangstingen ("Figure 2-8", side 9 i Atmel-ICE-userguide)
* Direkte, ved hjelp av kabelen nederst til høyre på "Figure 2-1", side 6 i Atmel-ICE-userguide.

Åkei, nå skal jeg prøve å forklare hvordan man kobler Atmel-ICE til brødbrettet:

1. Åpne side 1 av ATmega162-databladet (figuren med oversikt over pinner på mikroprosessoren)
2. Åpne side 14 av Atmel-ICE-userguide ("Table 3-1")
3. Hvis du bruker overgangen (altså ikke kobler direkte) så trenger du også "Figure 4-2" og "Table 4-1" på side 20 i Atmel-ICE-userguide.

Greia her er at tabellen på side 14 forteller hva Atmel-ICE spytter ut gjennom kabelen. De fleste av disse kablene er merket med pin-numre. Sjekk med noen andre hvis din kabel ikke er merket, så slipper du å koble feil. Vi bruker AVR, så se på "AVR port pin". Fra Atmel-ICE får man altså TCK ut på pin 1. Kult. Nå går vi til ATmega162-figuren. Her finner du TCK på pin 25. Fett. Pin 1 fra Atmel-ICE-kabelen skal altså kobles til på pin 25 på ATmega162. Gjør det samme med TDS, TDI og TDO. Koble GND-pinnene til jord. Drit i de pinnene som blir til overs.

Hvis du bruker overgangen istedenfor å koble direkte skal du gjøre akkurat det samme, men du må bruke tabellen og figuren på side 20 istedenfor. En ganske funky ting med "Figure 4-2" er at figuren er speilvendt-ish. Så hvis du ikke får ting til å funke kan du løse det ved å flippe figuren (altså bytte om kolonne 1 og 2).

#### L1 Oppgave 6

##### Kjekt å lese

* Hvis du er ny i Linux-verdenen, søk opp "Linux terminal cheat sheet". Hvis du ny med git (som jeg virkelig anbefaler alle å bruke), så lag en bruker på github og les en kom-i-gang-med-git-guide.
* Dokumentasjonen til avr-libc kan være greit å ha i bakhånd. Finnes på https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html
* "Register Summary", side 304 i ATmega162-databladet kan være kjekt å ha i bakhånd. Når du har kokt litt kode fra oppgavetekst, labforelesning eller databladet og du får feilmeldinger, er det ofte fordi registrene i koden ikke finnes på ATmega162-en. Ofte fordi f.eks. MCUCR heter MCUC0R i stedet. Da kan du finne de riktige registernavnene under "Register Summary" (eller bare ctrl+f registeret i databladet).
* "I/O ports", side 62 i ATmega162-databladet. Mye bra info på side 62-64.
* "Register descriptions for I/O ports", side 82 i ATmega162-databladet. Ikke stirr for mye på dette, da blir du unødvendig forvirra.
* "C Code Example"-ene i ATmega162-databladet. Det er 22 sånne der, bare skum gjennom noen. Da får du et inntrykk av hvordan man progger greia.


##### Kjekt å vite

Disclaimer: jeg bruker Linux. Jeg vet ikke om noe av det jeg skriver her funker på Windows.

Det første du må gjøre for å programmere ATmega162 er å laste ned alle pakkene du trenger. Kjør følgende i terminalen:

~~~bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install gcc-avr binutils-avr gdb-avr avr-libc avrdude
~~~

avr-libc er et bibliotek dere skal bruke for å progge mikrokontrolleren. Biblioteket blir antakelig lagret på /usr/lib/avr (eller bare søk på "avr" i filsystemet ditt). Inni der kan du blant annet finne *io.h* og *portpins.h* som kan være kjekt (men ikke nødvendig) å kikke litt på.

For å lage et lite testprogram trenger du også *Makefile*. Bruk den som blir lagt ut sammen med alle databladene. Øverst i *Makefile* må du endre til følgende:

~~~Makefile
# List all source files to be compiled; separate with space
SOURCE_FILES := main.c

# Set this flag to "yes" (no quotes) to use JTAG; otherwise ISP (SPI) is used
PROGRAM_WITH_JTAG := yes
~~~

Her har jeg altså tatt vekk alt annet enn `main.c` fordi testprogrammet bare skal inneholde *main.c*. Når du utvider programmet må de nye .c-filene legges til her (`SOURCE_FILES := main.c fil.c hei.c kaos.c`). I tillegg har jeg endret fra no til yes på PROGRAM_WITH_JTAG. Ganske enkelt fordi vi bruker JTAG.

Et enkelt testprogram kan se ut som dette:

*main.c*:
~~~c
#include <avr/io.h>
int main() {
  PORTB = 0b1;
  // De tre kommentere linjene under her er andre måter å skrive det samme som PORTB = 0b1
  // PORTB = 1;
  // PORTB = 0b00000001;
  // PORTB = (1 << PB0);

  return 0;
}
~~~

Programmet over inkluderer *io.h* fra avr-biblioteket. Det trengs for å forenkle kommunikasjonen med mikroprosessoren.

PORTB er definert av *io.h*. Ved å gi PORTB en verdi vil man sette pinnene på port B høye eller lave. Hvis du kikker på pin-oversikten til ATmega162 så ser du at port B er pinnene 1-8 (port A er pin 32-39, port C er pin 21-28 osv). `PORTB = 0b1`: her setter vi altså den første pinnen i port B til høy (1) og de andre pinnene til lav (0). Hvis vi i stedet skriver `PORTB = 0b01010101` vil vi sette annenhver pin på port B til høy og lav.

For å kompilere programmet og kjøre det må du først legge *main.c* og *Makefile* i samme mappe. Åpne deretter terminalen i mappen du har lagret greiene i og kjør følgende:

~~~bash
make
sudo make flash
~~~

`make` kompilerer programmet. Får du feilmeldinger her er det antakelig noe feil med C-koden din eller du har glemt å legge til filer i *Makefile*.
`sudo make flash` sender programmet ut til mikroprosessoren. (Det er ikke sikkert at du trenger å bruke `sudo` (`sudo` gjør at du utfører kommandoen som superbruker), men hvis `make flash` uten `sudo` gir feilmelding må du bruke `sudo make flash`.)

Gratulerer, du har nå programmert ATmega162! Sjekk at de riktige pinnene ble satt høye (5V) og lave (0V) med multimeter eller oscilloskopet :).

#### L1 Oppgave 7

##### Kjekt å lese

* "Alternate functions of port D", side 78 i ATmega162-databladet. Les om TXD0 og RXD0. (Obs: det finnes også TXD- og RXD-porter på port B. Disse heter TXD1 og RXD1, og info finnes under "Alternate functions of port B", side 72. Du skal bare bruke enten TXD/RXD0 eller TXD/RXD1. Vi brukte 0.)

##### Kjekt å vite

Databladet MAX220-49 dekker ganske mange MAX-er. Pass på at du finner den MAX-en som du har. Vi hadde MAX233, så denne beskrivelsen passer til MAX233.

Du finner den i "Figure 11" på side 21 i MAX-databladet. Både GND på pin 6 og GND på pin 9 må kobles til jord. Hvis du stirrer lenge nok på bildet til høyre i "Figure 11" kan du se at man også må koble sammen pin 11 med pin 15. Pin 10 må kobles sammen med pin 16. Pin 12 må kobles med pin 17. Pin 7 kobles til 5V (og husk decoupling-kondensator til jord som du ser øverst til høyre i "Figure 11").

Nå skal T1_in og R1_out på MAX kobles til henholdsvis TXD og RXD på ATmega162. På ATmega162 finnes det TXD0, RXD0, TXD1 og RXD1. Det er altså to transmit- og receive-kanaler på ATmega162. Vi valgte å bruke TXD0 og RXD0.

Så skal T1_out og R1_in på MAX kobles til henholdsvis Transmit (pin 3) og Receive (pin 2) på seriellkontakten. I tillegg skal Signal ground (pin 5) på seriellkontakten kobles til jord. Sjekk "Figure 10" på side 25 i oppgaveteksten for å finne riktige pinner på seriellkontakten. Obs! Det er lett å få dette speilvendt. Hvis det ikke funker, så er pin 5 den pinnen som du trodde var pin 1 osv.

#### L1 Oppgave 8

##### Kjekt å lese

* "Register Summary", side 304 i ATmega162-databladet er kjekt å ha i denne oppgaven også.
* "USART Initialization" m.m., side 172 i ATmega162-databladet. Greit å lese til og med side 177. Fra side 178 blir det unødvendig avansert og kronglete.

##### Kjekt å vite

Vi blåkopierte noe greier fra ATmega162-databladet. Hvis du søker på "usart_init" i databladet så finner du mye greier man kan lese sånn rundt side 172.

*uart.c*:
~~~c
#include <avr/io.h>
#include <stdio.h>
#include "uart.h"
void uart_Init(unsigned int ubrr) {
    /* Set baud rate */
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    /* Enable receiver and transmitter */
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    /* Set frame format: 8data, 2stop bit */
    UCSR0C = (1<<URSEL0)|(1<<USBS0)|(3<<UCSZ00);
}

void uart_Transmit( unsigned char data ) {
    /* Wait for empty transmit buffer */
    while ( !( UCSR0A & (1<<UDRE0)) );
    /* Put data into buffer, sends the data */
    UDR0 = data;
}
unsigned char uart_Receive( void ){
    /* Wait for data to be received */
    while ( !(UCSR0A & (1<<RXC0)) );
    /* Get and return received data from buffer */
    return UDR0;
}
~~~

Her ser dere at vi (til forskjell fra eksempelkodesnuttene i ATmega162-databladet) har puttet inn 0-tall i alle greiene. URSEL ble til URSEL0 osv. Hvis dere er ivrige etter mer info om akkurat det så går det an å søke i ATmega162-databladet etter f.eks. "UCSR0A" så kommer du til side 305 der du har en oversikt over hvilke registre som finnes på ATmega162.

#### L1 Oppgave 9

##### Kjekt å lese

* (Ikke nødvendig, men kanskje kjekt. Putty-guide: https://www.ssh.com/ssh/putty/putty-manuals/0.68/Chapter3.html)
* "C code example" under "USART Initialization", side 172 i ATmega162-databladet.
* "Internal Clock Generation – The Baud Rate Generator", side 169 i ATmega162-databladet.

##### Kjekt å vite

*main.c*:
~~~c
#include <avr/io.h>
#include "uart.h"

#define FOSC 4915200 //Klokkehastigheten på ATmega162. Altså klokkehastigheten på krystallen.
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1 //sjekk side 169 i ATmega162-databladet

void main(void) {
    uart_Init (MYUBRR);
    while (1) {
        uart_Transmit ('a');
    }
}
~~~

Denne sender 'a' fra mikroprosessoren til datamaskinen via seriellkabelen. Husk å oppdatere *Makefile* med `SOURCE_FILES := main.c uart.c`. Kjør `make` og `make flash` (eller `sudo make flash`) for å kompilere programmet og kaste det ut til mikroprosessoren.

Bruk putty for å lese seriellkommunikasjonen fra mikrokontrolleren. Stikkord her er baud: 9600, stoppbits: 2, port: /dev/ttyS0 (<- siste tegn er null, ikke o).
Putty kan startes via grafisk grensesnitt eller med `putty -serial /dev/ttyS0 -sercfg 9600,8,n,2,N` (?).

Hvis du ikke får opp "aaaaaaaaa..." i putty nå, så er det noe feil et sted. En grei måte å feilsøke på er å bruke oscilloskopet. Sjekk pinnen på ATmega162 som skal sende ut bokstaven. Prøv å kjøre programmet med og uten sending av 'a' sånn at du vet om prosessoren sender. Bruk oscilloskopet på MAX-en også. Og ved seriellkontakten. Hvis alt ser greit ut fram til seriellkontakten, så kan det tenkes at dere har koblet seriellkontakten motsatt av det som er rett.

#### L1 Oppgave 10

##### Kjekt å lese

* `fdevopen()`-dokumentasjonen på https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html#gab599ddf60819df4cc993c724a83cb1a4

##### Kjekt å vite

Dere skal putte et `fdevopen()`-kall inn i `uart_init()`-funksjonen-deres. Hvis dere kikker på `fdevopen()`-dokumentasjonen på https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html#gab599ddf60819df4cc993c724a83cb1a4 så ser dere at `fdevopen()` tar inn to funksjoner. Altså skal funksjonen kalles opp som følger:

~~~c
void uart_init(unsigned int ubrr) {
    /* Set baud rate */
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    /* Enable receiver and transmitter */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    /* Set frame format: 8data, 2stop bit */
    UCSR0C = (1 << URSEL0) | (1 << USBS0) | (3 << UCSZ00);

    // Ingenting over denne linjen er forandret

    fdevopen(uart_transmit, uart_receive);
}
~~~

Hvis dere kompilerer dette får dere antakelig noen snåle feilmeldinger. Løsningen på dette ligger i `fdevopen()`-dokumentasjonen. `fdevopen()` tar nemlig inn to funksjoner som har returtype `int` og som har argumenter at typen `char` og `FILE*`. Derfor må dere gjøre om `uart_transmit()` og `uart_receive()` til følgende:

~~~c
int uart_transmit(char data, FILE * file) {
    /* Wait for empty transmit buffer */
    while (! (UCSR0A & (1 << UDRE0)) );
    /* Put data into buffer, sends the data */
    UDR0 = data;

    return 0;
}

int uart_receive(FILE * file){
    /* Wait for data to be received */
    while ( !(UCSR0A & (1 << RXC0)) );
    /* Get and return received data from buffer */
    return UDR0;
}
~~~

Funksjonene er de samme, forskjellen er returtype `int`, at `uart_transmit()` returnerer `0`, og at begge funksjoner har fått et argument av typen `FILE*` som ikke gjør noe. Husk å oppdatere headerfila også.

#### L1 Oppgave 11

Nå kan *main.c* oppdateres til følgende:

~~~c
#include <avr/io.h>
#include "uart.h"

#define FOSC 4915200 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1 // 31

void main(void) {
  uart_init(MYUBRR);
  printf("Ting funker");
}
~~~

Kompiler og last opp til mikrokontrolleren. Kjør putty på PCen. Nå vil det (forhåpentligvis) dukke opp "Ting funker" i putty :).

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

### Lab 3

#### L3 Oppgave 1

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

#### L3 Oppgave 2

##### Verdt å lese

* Kapittel 2.2 i P1000-userguide (databladet for multifunksjonskortet)

##### Verdt å vite

I oppgaven står det "Make sure the jumper across “EXTSEL” is connected". For å få OLED til å vise sliderposisjonen må EXTSEL-jumperen tas av. Det står altså feil i oppgaven.

Det er litt samme hvilken spenningskilde (USB eller spenningsgenerator) man bruker til multifunk-kortet. Vi valgte å koble til den samme spenningen som breadboardet får fra spenningsgeneratoren for å få felles jord.

#### L3 Oppgave 3

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

#### L3 Oppgave 4

Pin 1 (?) (helt til venstre) på PORTB og pin 5 (?) (midten) på PORTD. Sjekk med oscilloskopet for sikkerhets skyld :).

#### L3 Oppgave 5

##### Verdt å lese

* Kapittel 3.8 i P1000-userguide

##### Verdt å vite

Sliderutgangene på PORTB og PORTD skal bare rett inn på FILTER-inngangene. FILTER-utgangene kobles på CH3 og CH4 på ADC.

#### L3 Oppgave 6

Touchknapp-pinnene er rett ved slider-pinnene på PORTB og PORTD. Husk også joystick-knappen på JOY-CONN pin 3. Vi koblet de tre knappene inn på PB0-PB2 på ATmega162.

#### L3 Oppgave 7-8

Se en eller annen kok.

### Lab 4

#### L4 Oppgave 1

Hvis dere har koblet opp og progget GALen som beskrevet i [lab 2 oppgave 3](lab2.md#l2-oppgave-3) så er alt good :).

#### L4 Oppgave 2

##### Verdt å lese

* "OLED interface", kapittel 3.1 i P1000-userguide
* "Figure 7.2", kp. 7.1.2 i SSD1700-databladet (om OLED)
* "Figure 13", side 28 i kapitlet "External memory interface" i ATmega162-databladet

##### Verdt å vite

Husk å koble på jumperen på EXTSEL for å styre OLED utenfra. Datainngangene på OLED-BUS (D1-D8) kan kobles rett på datautgangene på ATmega162 (PA0-PA7).

CS og D/C på OLED går rett til tilsvarende utganger på GALen.

WR (aka R/W) på OLED kan kobles rett på WR på ATmega162. Forklaring: I oppgaveteksten står det at OLEDen er hardvarekonfigurert til 8080-modus. I "Figure 7-2" i kp. 7.1.2 i SSD1780-databladet kan man se at data blir skrevet til OLEDen på rising edge på WR-signalet. Med andre ord kommer alt til å funke så lenge det fortsatt er data ute på databussen på det tidspunktet WR går fra lav til høy. I ATmega162-databladet i "Figure 13" ser man at ATmega162 sender data ut på databussen, og så blir WR lav, og så blir WR høy, og så slutter den å sende data. Altså er det fortsatt data på databussen når WR går fra lav til høy. Og derfor funker det å koble WR rett på WR :).

#### L4 Oppgave 3

Ja nei si det.

#### L4 Oppgave 4

##### Verdt å lese

* "Graphic display data ram", kp. 7.7 i SSD1780-databladet
* "Command table", kp. 8 i SSD1780-databladet. "Table 8-1".

##### Verdt å vite

OLEDen funker sånn at man skriver en byte (8 bit) til den om gangen. Det gjøres på akkurat samme måte som med SRAM. Man skriver ganske enkelt en byte til adressen som peker til OLEDen. Hvis dere (som oss) har tatt ugangspunkt i adresseoppsettet fra oppgaveteksten og progget GALen deretter (eller kokt GAL-programmet), så kan dere skrive en kommando til OLEDen ved å skrive til addresse `0x1000` og skrive data til OLEDen ved å skrive til adresse `0x1200`.

~~~c
void oled_write_command(uint8_t command) {
  volatile char *ext_oled_command = (char *) 0x1000;
  ext_oled_command[0] = command;
}

void oled_write_data(uint8_t data) {
  volatile char *ext_oled_data = (char *) 0x1200;
  ext_oled_data[0] = data;
}
~~~

Vi kokte OLED-initialiseringen rett fra databladet:

~~~c
void oled_init() {
 oled_write_command(0xae); // display off
 oled_write_command(0xa1); //segment remap
 oled_write_command(0xda); //common pads hardware: alternative
 oled_write_command(0x12);
 oled_write_command(0xc8); //common output scan direction:com63~com0
 oled_write_command(0xa8); //multiplex ration mode:63
 oled_write_command(0x3f);
 oled_write_command(0xd5); //display divide ratio/osc. freq. mode
 oled_write_command(0x80);
 oled_write_command(0x81); //contrast control
 oled_write_command(0x50);
 oled_write_command(0xd9); //set pre-charge period
 oled_write_command(0x21);
 oled_write_command(0x20); //Set Memory Addressing Mode
 oled_write_command(0x02);
 oled_write_command(0xdb); //VCOM deselect level mode
 oled_write_command(0x30);
 oled_write_command(0xad); //master configuration
 oled_write_command(0x00);
 oled_write_command(0xa4); //out follows RAM content
 oled_write_command(0xa6); //set normal display
 oled_write_command(0xaf); // display on
}
~~~

For å skjønne hvordan man skriver til OLEDen kan det lønne seg å ta en titt på "Figure 7-13" og "Figure 7-14" i kp. 7.7 SSD1780-databladet. OLEDen er altså delt inn i 8 linjer (PAGE0-PAGE7) og 128 kolonner (SEG0-SEG127). Hver kolonne i hver page består av 8 bit (D0-D7).

For å få fram noe på OLEDen sender man først en kommando for å fortelle hvor man på OLEDen man skal skrive, og deretter sender man 8 bit til OLEDen.

Horizontal, vertical og page addressing mode er ulike moduser man kan velge som på litt ulike måter lar deg bestemme posisjon på OLEDen. Vi brukte stort sett page addressing mode, så denne beskrivelsen tar utgangspunkt i at du bruker det.

I "Table 8-1" finnes info om hvordan man skriver kommandoer og data til OLEDen. For å  velge modus må man sende kommandoen `0x20` aka `0b00100000` til OLEDen. Deretter sender man `0b10` for å velge page addressing mode:

~~~c
// For å skru på page addressing mode:
oled_write_command(0x20);
oled_write_command(0b10);

// For å skru på horizontal addressing mode:
oled_write_command(0x20);
oled_write_command(0b00);

// For å skru på vertical addressing mode:
oled_write_command(0x20);
oled_write_command(0b01);
~~~

Litt lenger ned i tabellen står det hvordan man forteller OLED hvilken linje (aka PAGE0-PAGE7) man vil skrive til. For å velge linje nummer `n` må man skrive kommandoen `0xB0 + n`.

~~~c
oled_write_command(0xB0); // For å gå til linje 0

oled_write_command(0xB3); // For å gå til linje 3
~~~

Man må også bestemme seg for hvilken kolonne man vil skrive til. Man velger en kolonne mellom 0 og 127. For å fortelle OLED hvilken kolonne man velger må man bruke to kommandoer (se øverst i tabellen). Man må sende "lower nibble of the column start address" og "higher nibble...". Lower nibble er de fire minst signifikante bitene, higher nibble er de fire mest signifikante bitene.

For eksempel, hvis man vil velge kolonne 97 (`0b01100001`) vil lower nibble være `0001` og higher nibble være `0110`. Så for å fortelle OLED at man vil velge kolonne 97 gjør man følgende:

~~~c
oled_write_command(0x00 + 0b0001); // Setter lower nibble for kolonne 97
oled_write_command(0x10 + 0b0110); // Setter higher nibble for kolonne 97
~~~

For å finne lower og higher nibble basert på kolonnenummer kan man bruke de freshe greiene heltallsdivisjon og modulo. Med kolonne 97 som eksempel fremdeles: \
`97 / 16 = 6 aka 0b0110 (higher nibble)` \
`97 % 16 = 1 aka 0b0001 (lower nibble)`

Du ender antakelig opp med noe som ligner på det her:

~~~c
void oled_goto_column(int column) {
  oled_write_command(0x00 + (column % 16)); // Lower nibble
  oled_write_command(0x10 + (column / 16)); // Higher nibble
}
~~~

Når man endelig har fått satt riktig addressing mode, gått til riktig linje og gått til riktig kolonne, så kan man skrive litt data til OLEDen. Man skriver da 8 bit. Se "Figure 7-14" i SSD1780-databladet, så ser du at hver byte tilsvarer en vertikal linje hvor høye bit betyr at pikselen skal lyse, mens lave bit betyr av pikselen skal skrus av.

En litt stilig greie med OLEDen er at når du har gått til en linje og en kolonne og skrevet en byte til skjermen (aka skrudd på noen piksler i den vertikale 8-bit linja), så økes kolonneposisjonen din automatisk med én. Det vil si at hvis du går til linje nummer 3, kolonne nummer 10 og så skriver data `0b1` til OLED 20 ganger rett etter hverandre (uten å gå til en ny linje eller kolonne innimellom), så får du en horisontal rett linje på OLED-skjermen som er 20 piksler lang.

For å gjøre hele skjermen blank/sette alle pikslene lave kan man loope over alle de 8 linjene på skjermen, og deretter skrive `0` til den 128 ganger. Man trenger ikke å flytte seg bortover 127 ganger ettersom OLEDen automatisk flytter posisjonen din én kolonne bort hver gang man skriver data.

~~~c
// Setter alle piksler på skjermen lave
for (int line = 0; line < 8; line++) {
	oled_goto_line(line);
	oled_goto_column(0);
	for (int i = 0; i < 128; i++) {
		oled_write_data(0x00);
	}
}
~~~

#### L4 Oppgave 5

Last ned *fonts.h* fra Blackboard. Trenger ikke gjøre noe mer.

#### L4 Oppgave 6

Vi droppet å bruke `printf()` og laget heller en egen `oled_print()`. Hvis man stirrer litt på innholdet i *fonts.h*, så skjønner man kanskje at hvert element i font-arrayet inneholder noen byte-elementer som til sammen utgjør en bokstav. For eksempel "A", hentet fra font8-variablen:

~~~
{0b01111100,0b01111110,0b00010011,0b00010011,0b01111110,0b01111100,0b00000000,0b00000000}, // A
~~~

Under her står hvert av elementene i A-arrayet nedover, med det minst signifikante bitet øverst. Hvis man legger godvilja til og ser på 1-tallene, så minner det om en "A". Det er altså sånn man må skrive bitene/pikslene til OLEDen for å få fram "A"-en.

~~~
0011000
0111100
1100110
1100110
1111110
1100110
1100110
0000000
~~~

Når man skal skrive A-en ut på skjerman kan man ganske enkelt iterere over elementene i A-arrayet og skrive hvert element til skjermen etter hverandre. Det funker fordi det minst signifikante bitet man skriver til OLEDen havner lengst opp på skjermen (akkurat som A-illustrasjonen over) og fordi OLED-en automagisk flytter oss en kolonne videre for hver byte vi sender.

Så for å skrive A fra font8 kan man gjøre sånn her omtrent:

~~~c
int A = 33; // Fordi A-bytene er på plass 33 i font8-arrayet
for (int i = 0; i < 7; i++) { // Fordi font8 består av sju byte per bokstav
	int byte = pgm_read_byte(&font8[A][i]); // Henter data fra PROGMEM
	oled_write_data(byte);
}
~~~

Hvis man sammenligner hvilke plasser bokstavene har i font-arrayene med bokstavenes ASCII-kode (samme kode som `char`-typen bruker), så finner man ut at det kan være fornuftig å gjøre noe sånt som dette:

~~~c
void oled_print_char(char c) {
  c = c-32;
  for (int i = 0; i < 7; i++) {
    int byte = pgm_read_byte(&font8[c][i]);
    oled_write_data(byte);
  }
}
~~~

Da kan man skrive en bokstav til skjermen ved å bruke for eksempel `oled_print_char("h")`.

For å skrive ut strenger aka mange `char` må man bruke `char`-array og ei litta loop. `strlen()` gir lengden på en tekststreng.

~~~c
void oled_print(char c[]) {
  for (int i = 0; i < strlen(c); i++) {
    oled_print_char(c[i]);
  }
}
~~~

Da kan man skrive tekst til OLED-skjermen ved å bruke `oled_print("Livet er et kaffe")`.

#### L4 Oppgave 7

Stikkord for å mekke meny selv: menyelement-struct med felter for menytittel, funksjonspeker, array av pekere til undermenyelementer, og peker til "overmenyen" (altså menyen man kom fra hvis dette elementet er en undermeny).

Hvis man er i en meny og velger et menyelement som ikke har en undermeny, så kalles funksjonen som funksjonspekeren peker til.

Hvis man er trykker på et menyelement som har en undermeny, så går man til undermenyen.

Hvis man går tilbake fra en undermeny, så går man til den menyen som er lagret i "peker til overmeny"-feltet i structen.

Om man gjør det på en sånn-ish måte så kan man ha evig mange menyelementer i hver meny og så mange underunderundermenyer som man orker.

Og så trenger man noe greier for å finne ut hvilken retning joysticken beveges. Lag en funksjon som finner ut hvilken retning (x eller y) som har størst utslag i positiv eller negativ retning, deretter finn ut om utslaget i en av retningene er større enn et eller annet tall. Så returnerer dere en enum-verdi (eller bare en int) når dere vet hvilken retning joysticken presses i.

Menyelement-structen vår ser sånn her ut:

~~~c
#define MAX_SUBMENUS 10 // Må settes til det høyeste antall menyelementer man har tenkt til å ha i samme meny
typedef struct Menu {
	char * text; // Teksten/tittelen til menyelementet
	void (*function)(); // Peker til funksjonen som skal kalles hvis det ikke finnes noen undermeny
	struct Menu * parent; // Pappameny :)
	struct Menu * subMenu[MAX_SUBMENUS]; // Undermeny. Array av pekere til menyelementer
} menu_t, *menu_ptr;
~~~

Da kan vi initialisere en tom meny med noe sånt som det under. Her lager vi bare et helt tomt menyelement. Tanken er at de menyelementene som vises når man starter menyen, er undermenyelementene til denne tomme menyen.

~~~c
menu_ptr menu = malloc(sizeof(menu_t));
~~~

Nå trengs en funksjon for å legge til undermenyelementer. Undermenyelementene skal ikke være tomme, de skal inneholde tittel, funksjonspeker, peker til pappamenyen, og evt. et array av pekere til undermenyelementer.

Funksjonen `menu_add()` under tar inn argumentene tittel, peker til foreldremenyen og funksjonspeker, og så opprettes et menyelement med verdier fra argumentene. Deretter blir en peker til menyelementet lagt til på slutten av foreldremenyen sitt undermeny-array.

~~~c
menu_ptr menu_add(menu_ptr parent, char * text, void (*function)()) {
	menu_ptr subMenu = malloc(sizeof(menu_t)); // Oppretter tomt undermenyelement
	subMenu->text = text; // Setter inn tittel
	subMenu->function = function; // Setter inn peker til funksjon
	subMenu->parent = parent; // Setter inn peker til foreldremenyen

	// Legger undermenyelementet til slutten av undermeny-arrayet til foreldremenyen
	int i = 0;
	while (parent->subMenu[i] != NULL) {
		i++;
	}
	parent->subMenu[i] = subMenu;

	return subMenu; // Returnerer en peker til undermenyelementet vi har opprettet
}
~~~

Kult. Nå kan vi lage en meny. For å lage en meny med to elementer, hvor det ene elementet utfører en funksjon og det andre elementet går til en annen undermeny med tre elementer så kan man gjøre sånn her:

~~~c
menu_ptr menu = malloc(sizeof(menu_t)); // Selve menyen initialiseres

// Undermenyelementer til menu. Disse fungerer som hovedmeny
// Legg merke til at "Spill et spill"-menyyen får funksjonspeker NULL fordi denne skal ha en undermeny
menu_ptr menu_highscore = menu_add(menu, "Vis highscore", &show_highscore);
menu_ptr menu_play = menu_add(menu, "Spill et spill", NULL);

// Undermenyelementer til "Spill et spill"-menyen. Hvert element peker til en funksjon som starter et spill
menu_ptr menu_game1 = menu_add(menu_play, "Spill 1", &play_game1);
menu_ptr menu_game2 = menu_add(menu_play, "Spill 2", &play_game2);
menu_ptr menu_game3 = menu_add(menu_play, "Spill 3", &play_game3);

// Funksjonene som blir kalt når man trykker på riktig menyelement
void show_highscore() { /* Viser highscore */ }
void play_game1() { /* Spiller spill 1 */ }
void play_game2() { /* Spiller spill 2 */ }
void play_game3() { /* Spiller spill 3 */ }
~~~

Med greiene over får vi altså en meny som ser ut som dette:

~~~
"Vis highscore"  -> show_highscore();
"Spill et spill" -> "Spill 1" -> play_game1();
                    "Spill 2" -> play_game1();
                    "Spill 3" -> play_game1();
~~~

Og på samme måte kan legge til undermenyelementer til for eksempel "Spill 2"-menyen og enda en undermeny under der igen og så videre.

Det vi har nå er altså en masse structer som peker til hverandre. Det som står igjen nå er logikk for å navigere mellom structene og for å vise elementene som er i en meny. Skriver kanskje litt om det sånn etterhvert en gang

### Fortsettelse følger

---
TTK4155 \
Byggern \
Industrielle og innbygde datasystemers konstruksjon \
Embedded and Industrial Computer Systems Design \
NTNU
