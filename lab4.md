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
