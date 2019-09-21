### Lab 1

#### Oppgave 1

##### Kjekt å lese

* Figurene på første side av LM7805-databladet

##### Kjekt å vite

Ta utgangspunkt i "Figure 3 - breadboard" fra oppgaveteksten. Det har ikke noe å si om dere har fire rader på breadboardet istedenfor tre eller om spenningskontaktene på siden står motsatt vei.

Det kan lønne seg å begynne med å plassere alt sånn som det er vist i "Figure 5" fra oppgaveteksten. Da slipper dere å rive opp alt dere har koblet senere fordi dere må få plass til en ny komponent :).

Spenningsregulatoren LM7805 skal kobles opp sånn som det står til høyre på første side i LM7805-databladet. Vi brukte 100 µF (?) på kondensatoren på output.

Hvis dere kobler høy spenning (10-12 V) rett inn på komponentene deres, så risikerer dere å ødelegge komponentene. Om dere gjør det er det bare å hente ny komponent, men det er like greit å bare ikke drepe komponentene. Så jeg anbefaler alle å ha forsyningsspenning (10-12V) på én av spenningslinjene på breadboardet, og så sette opp 5V på alle de andre spenningslinjene med én gang. (Vi klarte å koble SRAM-brikken til 10V og brukte litt tid på å finne ut om den fortsatt funket.)

#### Oppgave 2

##### Kjekt å lese

* Om decoupling capacitors i oppgaveteksten. Andre avsnitt under "Decoupling", side 10.
* "Figure 1", side 1 i ATmega162-databladet. Oversikt over pinnene på ATmega162.

##### Kjekt å vite

5V fra spenningsregulatoren skal inn på VCC på ATmega162. GND til jord.

"Decoupling capacitor" kan være 100 µF (?). Kobles mellom pin 1 (VCC) på ATmega162 og jord.

#### Oppgave 3

##### Kjekt å lese

* Kapittel 2 ("Connection of reset pins on AVR") i AVR-userguide. Les hele siden, inkludert underkapittel 2.2
* Les om "RESET" under "Pin descriptions", side 6 i ATmega162-databladet.

##### Kjekt å vite

Sjekk side 5-6 i AVR-userguide. Kort fortalt skal dere koble opp kretsene som er i figurene på side 6.

Punktet der det står "RESET" i "Figure 2-3" skal kobles på der det står "Reset" i "Figure 2-2". "SW" er en trykknapp. Husk å måle på knappen hvilken vei den kortsluttes når man trykker på den. Ikke tenk på de rare linjene på høyre og venstre side av "SW" i "Figure 2-3".

Reset på ATmega162 er pin 9.

#### Oppgave 4

##### Kjekt å lese

* Søk på "crystal" i AVR-userguide. Ikke så mye interessant her, men kan hjelpe på forståelsen av hva en krystall er.
* Les under "Crystal Oscillator", side 36 i ATmega162-databladet. "Figure 19" viser koblingsoppsettet.

##### Kjekt å vite

Krystallen dere skal bruke er en av de to små sølvfargede rare greiene. De har ulike frekvenser, og frekvensen på krystallen står på toppen av den. Den som det står 4.9152 på skal kobles mellom pin 19 (XTAL1) og pin 18 (XTAL2) på ATmega162. Og så kondensatorer fra hvert bein på krystallen til jord.

#### Oppgave 5

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

#### Oppgave 6

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

#### Oppgave 7

##### Kjekt å lese

* "Alternate functions of port D", side 78 i ATmega162-databladet. Les om TXD0 og RXD0. (Obs: det finnes også TXD- og RXD-porter på port B. Disse heter TXD1 og RXD1, og info finnes under "Alternate functions of port B", side 72. Du skal bare bruke enten TXD/RXD0 eller TXD/RXD1. Vi brukte 0.)

##### Kjekt å vite

Databladet MAX220-49 dekker ganske mange MAX-er. Pass på at du finner den MAX-en som du har. Vi hadde MAX233, så denne beskrivelsen passer til MAX233.

Du finner den i "Figure 11" på side 21 i MAX-databladet. Både GND på pin 6 og GND på pin 9 må kobles til jord. Hvis du stirrer lenge nok på bildet til høyre i "Figure 11" kan du se at man også må koble sammen pin 11 med pin 15. Pin 10 må kobles sammen med pin 16. Pin 12 må kobles med pin 17. Pin 7 kobles til 5V (og husk decoupling-kondensator til jord som du ser øverst til høyre i "Figure 11").

Nå skal T1_in og R1_out på MAX kobles til henholdsvis TXD og RXD på ATmega162. På ATmega162 finnes det TXD0, RXD0, TXD1 og RXD1. Det er altså to transmit- og receive-kanaler på ATmega162. Vi valgte å bruke TXD0 og RXD0.

Så skal T1_out og R1_in på MAX kobles til henholdsvis Transmit (pin 3) og Receive (pin 2) på seriellkontakten. I tillegg skal Signal ground (pin 5) på seriellkontakten kobles til jord. Sjekk "Figure 10" på side 25 i oppgaveteksten for å finne riktige pinner på seriellkontakten. Obs! Det er lett å få dette speilvendt. Hvis det ikke funker, så er pin 5 den pinnen som du trodde var pin 1 osv.

#### Oppgave 8

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

#### Oppgave 9

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

#### Oppgave 10

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

#### Oppgave 11

##### Kjekt å vite

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
