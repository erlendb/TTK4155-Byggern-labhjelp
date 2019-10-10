### Lab 5

#### L5 Oppgave 1

##### Verdt å lese

* Oversikt over pinner, første side i MCP2515-databladet

##### Verdt å vite

VSS til jord. VDD til 5V med decoupling-kondensator.

Den krystallen dere har igjen skal kobles på MCP2515 mellom OSC1 og OSC2, og så må både OSC1 og OSC2 kobles til jord. Krystallen gir MCP2515 klokkesignal.

RESET kan kobles på den samme resetknappen som dere bruker til å resette ATmega162.


#### L5 Oppgave 2

MCP2515 skal kobles til pin 5-9 (SPI-pinnene) på ATmega162. SCK går på SCK. SO (Slave out) går på MISO (Master in, slave out). SI (Slave in) går på MOSI (Master out, slave in).

CS (Chip select) går på SS (Slave select). For å få SS-signalet riktig koblet vi en pull-up-motstand på CS, altså en stor motstand fra CS til 5V (i tillegg til koblingen fra SS til CS).

På ATmega162 finnes tre interrupt-innganger. Vi brukte INT0. Vi koblet med andre ord INT på MCP2515 inn på INT0 på ATmega162.

#### L5 Oppgave 3

##### Verdt å lese

* "Serial Peripheral Interface – SPI", side 157-161 i ATmega162-databladet.
* "SS pin functionality", side 162-164 i ATmega162-databladet.

##### Verdt å vite

SPI-driveren bør inneholde en initialiseringsfunksjon og funksjoner for å lese fra og skrive til SPI. Vi kokte det meste fra ATmega162-databladet, men med noen endringer. Det er mulig å lage interruptdrevet SPI-driver, men vi droppet det og kjørte på med polling av statusregisteret.

Først kan det være kjekt å definere hvilke pinner og porter (på ATmega162) som brukes i driveren. Dette kommer godt med når dere senere skal tilpasse SPI-driveren til node 2.

~~~c
#define DDR_SPI DDRB
#define DD_SS PB4
#define DD_MOSI PB5
#define DD_MISO PB6
#define DD_SCK PB7
~~~

I SPI-driveren og i MCP-driveren kommer du til å få mye bruk for å sette SS (aka CS på MCP2515) høy og lav, så det kan være kjekt å ha følgende to funksjoner i bakhånd:

~~~c
// Setter SS høy. Lar alt annet på PORTB være som det var.
void spi_set_ss() {
	PORTB |= (1 << DD_SS);
}

// Setter SS lav. Lar alt annet på PORTB være som det var.
void spi_clear_ss() {
	PORTB &= ~(1 << DD_SS);
}
~~~

SPI er laget sånn at én komponent er master, mens en annen er slave. Vi vil ha ATmega162 som master.

~~~c
void spi_master_init() {
	// Setter følgende som utganger: MOSI, SCK, SS. Resten av PORTD settes som innganger.
	DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS);

	// Skrur på SPI. Setter ATmega162 som master. Setter klokkefrekvensen til fck/16. (SPIE?)
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPIE);

	// Setter SS (aka CS på MCP2515) høy.
	spi_set_ss();
}
~~~

**MOSI** står som sagt for Master out slave in, så den settes som utgang fordi ATmega162 er master. MISO (Master in, slave out) blir i funksjonen over satt som inngang av samme grunn. **SCK** er klokkefrekvensen som SPI-kommunikasjonen baseres på. ATmega162 er master, og derfor er det ATmega162 sin klokkefrekvens som bestemmer tempoet til SPIen. Og derfor er SCK utgang. **SS** er signalet ATmega162 sender til MCP2515 for å aktivere MCP2515 og må derfor være utgang.

I SPCR (SPI control register) setter vi **SPE** (SPI enable) for å skru på SPI. **MSTR** settes for å gjøre ATmega162 til master. Ved å sette **SPR0** blir SPI-klokkefrekvensen satt til en 16-del av ATmega162-klokkefrekvensen. **SPIE** (SPI interrupt enable) settes høy for å kunne motta interrupt når vi er ferdig med å skrive til SPI.

**NB!** Det er viktig å sette SS som utgang FØR man setter ATmega162 som master. Hvis man setter ATmega162 som master mens SS er en inngang, så blir ATmega162 satt som slave hvis SS blir lav. Hvis SS derimot er en utgang, så forblir ATmega162 master uansett om SS er høy eller lav.

For å skrive til SPI gjør man akkurat som databladet sier:

~~~c
void spi_write(char cData) {
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF))) {
	}
}
~~~

For å lese fra SPI må man sende en dummy-byte ut på SPIen. Hvorfor? Ingen vet. Men det har noe å gjøre med at SPI alltid skriver og leser samtidig eller noe sånt. Les om "full duplex bus" hvis du er nysgjerrig.

Man skulle kanskje tro at det ga mening å skrive `0` til SPDR (SPI data register) før man leste, men der tok vi visst feil alle sammen. Ifølge Waseem (vitass) kan det skape problemer å sende ut `0`, og man bør heller sende ut `0xFF` som dummy-byte.

Når byten er sendt ut på SPI er det bare å lese i vei :).

~~~c
uint8_t spi_read() {
	SPDR = 0xFF;
	/* Wait for reception complete */
	while(!(SPSR & (1<<SPIF))) {
	}
	/* Return data register */
	return SPDR;
}
~~~

I lese- og skrive-funksjonene over finnes `while(!(SPSR & (1<<SPIF))) {}`. Den whilen sjekker om SPIF (SPI interrupt flag) i SPSR (SPI status register) er satt. SPIF blir høy når en SPI-overføring er fullført.

#### L5 Oppgave 4

##### Verdt å lese

* Hele kapittel 12, side 65-70 i MCP2515-databladet, inkludert figurer og timing-diagrammer.

* Kapittel 10 "Modes of operation", side 59- i MCP2515-databladet, inkludert CANCTRL- og CANSTAT-registrene

* Kapittel 11 "Register map" i MCP2515-databladet

##### Verdt å vite

Oppgaven sier at du skal implementere følgende: Read, Write, Request to send, Read status, Bit modify, Reset.

Reset er viktig. Denne fungerer som initialisering av MCP-greiene.

Read, write og request to send må man ha for å sende ting med CAN.

Bit modify er nødvendig for å sette bit i kontrollregistrene til MCP.

Read status trenger du strengt talt ikke å bry deg med.

Det kan også være kjekt å lage en `mcp_set_mode()`-funksjon.

For å implementere alle disse greiene trenger du registrene som er definert i *MCP2515.h*-fila. Finnes på Blackboard eller i en kok nær deg.

MCP-initialiseringen kokte vi fra labforelesning:
~~~c
// Init kokt rett fra Waseem
void mcp_init() {
	spi_master_init();
	mcp_reset(); //Se kode for denne lenger ned

	_delay_ms(1); //Viktig!

	// Sjøltesting
	uint8_t value = mcp_read(MCP_CANSTAT); //Se kode for denne lenger ned
	if ((value & MODE_MASK) != MODE_CONFIG) {
		printf("MCP2515 er ikke i konfigurasjonsmodus etter reset. CANSTAT: %x \r\n", value);
	}
}
~~~

Funksjonen over initialiserer SPI før den resetter (aka initialiserer) MCP. Så har vi et bittelite delay, før vi sjekker hvorvidt MCP-en er i konfigurasjonsmodus.

Kapittel 12.2 i MCP2515-databladet forteller oss at "it is highly recommended that the Reset command be sent (or the RESET pin be lowered) as part of the power-on initialization sequence". Så derfor resetter vi MCP-en i init-funksjonen :).

MCPen skal gå inn i konfigurasjonsmodus rett etter resetting. Men det viser seg at det kan oppstå litt krøll her hvis man ikke legger inn et delay etter `spi_master_init()` og `mcp_reset()`. Uten delay kan `mcp_read(MCP_CANSTAT)` finne på å returnere den verdien du har i spi-dummy-byten din (antakelig `0xFF`). Med ei litta delay er det større sjanse for å få riktig returverdi fra `mcp_read(MCP_CANSTAT)`. Når MCP-en er i konfigurasjonsmodus skal MCP_CANSTAT inneholde `0x80`. Når MCP er i normalmodus (MODE_NORMAL) skal CANSTAT inneholde `0x00`. Når MCP er i loopbackmodus (MODE_LOOPBACK) skal CANSTAT inneholde `0x40`.

For å skrive, lese, resette, bitmodifisere og alt mulig sånt så anbefaler jeg sterkt at dere ser ekstra godt på timingdiagrammene og tabellene i kapittel 12 i MCP2515-databladet.

For å resette MCP-en sender man ganske enkelt MCP_RESET-instruksjonen til MCP-en. MCP_RESET er definert til `0xC0` i *MCP2515.h*. Kapittel 12.2 i MCP2515-databladet kan fortelle oss følgende:
"The RESET instruction is a single-byte instruction that requires selecting the device by pulling CS low, sending the instruction byte and then raising CS. Ålreit, kult. Da gjør vi det, da:

~~~c
void mcp_reset() {
	spi_clear_ss(); // "selecting the device by pulling CS low,"
	spi_write(MCP_RESET); // "... sending the instruction byte"
	spi_set_ss(); // "... and then raising CS"
}
~~~

Lese-, skrive-, statuslesing- og bitmodifiseringsfunksjonene implementeres på tilsvarende måte. Implementeringen følger ganske enkelt timing-diagrammene i kapittel 12 i MCP2515-databladet.

`MCP_READ`, `MCP_WRITE`, `MCP_READ_STATUS` og så videre er instruksjoner som er definert i *MCP2515.h*-fila. Om man kikker i "TABLE 12-1: SPI INSTRUCTION SET" i MCP2515-databladet så ser man at instruksjonskonstantene som er definert i *MCP2515.h* stemmer.

~~~c
uint8_t mcp_read(uint8_t address) {
	spi_clear_ss();
	spi_write(MCP_READ);
	spi_write(address); //Adressen (på MCP2515) som vi vil lese fra
	uint8_t data = spi_read();
	spi_set_ss();

	return data;
}

void mcp_write(uint8_t address, uint8_t data) {
	spi_clear_ss();
	spi_write(MCP_WRITE);
	spi_write(address); //Adressen vi vil skrive til
	spi_write(data);
	spi_set_ss();
}

char mcp_read_status() {
	spi_clear_ss();
	spi_write(MCP_READ_STATUS);
	char data = spi_read();
	spi_set_ss();

	return data;
}

void mcp_bit_modify(uint8_t address, uint8_t mask, uint8_t data) {
	spi_clear_ss();
	spi_write(MCP_BITMOD);
	spi_write(address); //Adressen der vi vil endre en eller flere bit
	spi_write(mask); //Maskeringsbyte, se forklaring nedenfor
	spi_write(data); //Verdiene som biten(e) skal endres til
	spi_set_ss();
}
~~~

Litt ekstra om bit modify: \
Her sender man først BITMOD-instruksjonen som forteller at nå skal vi modifisere en eller flere bit. Så sender man adressen til den byten som man vil modifisere bit i. Deretter sender man en maskeringsbyte, før man sender dataen som bitene skal endres til.

Maskeringsbyten (f.eks `0b00001100`) må ha 0 for bitene som ikke skal forandres og 1 for bitene som skal forandres.

La oss si at vi vil endre bit 2 og bit 3 i CANCTRL-registeret til 0, men vi vil at alle andre skal være som de var. Da må følgende gjøres:
~~~c
spi_clear_ss();
spi_write(MCP_BITMOD);
spi_write(MCP_CANCTRL);
spi_write(0b00001100); // maskerer alle bitene bortsett fra bit 2 og bit 3
spi_write(0b00000000); // setter bit 2 og bit 3 til 0. Alt annet blir som det var fra før.
spi_set_ss();
~~~

Vi lagde en request to send-funksjon som takler alle mulige buffernumre. I praksis kommer du antakelig til å kun sende fra buffer 0. Funksjonen under her passer på at hvis input-buffer-nummeret er større enn 2 (fordi vi har tre sende-buffere på MCP2515), så settes buffernummeret ned til en buffer vi har tilgjengelig.

Deretter sender funksjonen en request to send-instruksjon til MCP2515 via SPI. Med request to send-instruksjonen ber vi MCP2515 om å ta det som er på gitt buffer og sende det ut på CAN-bussen.

~~~c
void mcp_request_to_send(int buffer_number) {
	spi_clear_ss();
	buffer_number = buffer_number % 3; // Mapper buffernummer til 0, 1, 2
	char data = MCP_RTS_TX0;
	if (buffer_number == 0) {
		data = MCP_RTS_TX0;
	} else if (buffer_number == 1) {
		data = MCP_RTS_TX1;
	} else if (buffer_number == 2) {
		data = MCP_RTS_TX2;
	}
	spi_write(data);
	spi_set_ss();
}
~~~

En enklere variant av funksjonen over som kun lar deg bruke buffer nr. 0 kan implementeres sånn her:

~~~c
void mcp_request_to_send_buffer0() {
	spi_clear_ss();
	char data = MCP_RTS_TX0;
	spi_write(data);
	spi_set_ss();
}
~~~

Funksjonen under her gjør det enkelt å sette MCP-en i riktig modus. Moduser er allerede definert i MCP2515-h-fila (f.eks. `MODE_NORMAL` og `MODE_LOOPBACK`), så for å sette MCP-en i loopbackmodus så kan du skrive `mcp_set_mode(MODE_LOOPBACK);`. Vi bruker bit modify her fordi det kun er tre av bitene i CANCTRL-registeret som bestemmer modusen, og vi ønsker å beholde resten av CANCTRL som det var fra før. (Se "REGISTER 10-1: CANCTRL" i MCP2515-databladet.)

~~~c
void mcp_set_mode(uint8_t mode) {
	mcp_bit_modify(MCP_CANCTRL, 0b11100000, mode);
}
~~~

Nå har du funksjonene du trenger. For å sende og motta med MCP trengs også adressene til bufrene som skal brukes. MCP2515 har tre sendebufre og to mottaksbufre.

Om man ser litt hardt på "TABLE 11-1" i kapittel 11 i MCP2515-databladet ser man at `TXB0SIDH` finnes på adressen `0b0011 0001`. Om man ser på tabellen ved siden av "FIGURE 12-5: Load TX buffer", side 68, ser man at nettopp `TXB0SIDH` er starten av "TX buffer 0". TX-buffer 0 er det samme som sendebuffer nr. 0. I samme tabell kan man også se at sendebuffer 1 starter på `TXB1SIDH` og sendebuffer 2 starter på `TXB2SIDH`. I "FIGURE 12-3: READ RX BUFFER INSTRUCTION" finner man samme greier for mottaksbufrene. Man ender opp med følgende (som kan være greit å definere i *MCP2515.h*):

~~~c
// Send-buffer 0
#define MCP_TXB0SIDH 0x31 // aka 0b0011 0001

//Send-buffer 1
#define MCP_TXB1SIDH 0x41

//Send-buffer 2
#define MCP_TXB2SIDH 0x51

// Motta-buffer 0
#define MCP_RXB0SIDH 0x61

// Motta-buffer 1
#define MCP_RXB1SIDH 0x71
~~~

Både sende- og mottaksbufrene består av en hel del mer enn bare "SIDH", men det er ikke verdt å tenke på det før CAN-driveren skal implementeres i neste oppgave.

Nå kan SPI og MCP testes ved å sette MCP i loopbackmodus. Som det står i kapittel 10 i MCP2515-databladet kan MCP-en kun settes i en modus når MCP-en står i konfigurasjonsmodus. MCP-en går i konfigmodus rett etter resetting, så vi kan sette den i loopbackmodus etter initialisering/resetting.

~~~c
uart_init(UBRR);

// Initialiserer SPI og MCP:
//spi_master_init(); //Ta med denne hvis ikke spi_master_init() ligger inni mcp_init()
mcp_init();

// Setter MCP i loopbackmodus og sjekker CANSTAT:
mcp_set_mode(MODE_LOOPBACK);
printf("mode: %x\r\n", mcp_read(MCP_CANSTAT));
// Når MCP står i loopbackmodus skal CANSTAT være 0b01000000 aka 0x40

// Skriver en tilfeldig byte (0xA7) til MCP-sendebuffer0 og leser fra mottaksbuffer0
// Her bør man lese det samme som man sender så lenge MCPen står i loopbackmodus.
mcp_write(MCP_TXB0SIDH, 0xA7); // Skriver 0xA7 til sende-buffer nr. 0
mcp_request_to_send(0); // Sender 0xA7 fra bufferen ut på CAN-bussen
uint8_t byte = mcp_read(MCP_RXB0SIDH); // Leser fra mottaksbuffer nr. 0
printf("mottar: %x\r\n", byte); //Skal være samme som man sender, altså 0xA7
~~~

#### L5 Oppgave 5

##### Verdt å lese

* Kapittel 3.1 "Transmit buffers" i MCP2515-databladet
* "Table 11-1" i kapittel 11 i MCP2515-databladet
* Registrene i kapittel 3 "Message transmission" i MCP2515-databladet
* Registrene i kapittel 4 "Message reception" i MCP2515-databladet

##### Verdt å vite

CAN-driveren trenger i først omgang en initialiseringsfunksjon, en sendefunksjon og en mottaksfunksjon. Du bør også lage en struct som definerer strukturen på meldingene du skal sende med CAN.

Initialiseringsfunksjonen trenger ikke å gjøre noe mer fancy enn å initialisere SPI og MCP:

~~~c
void can_init() {
	//spi_master_init(); //Ta med denne dersom den ikke ligger inni mcp_init()
	mcp_init();
}
~~~

For å forstå hvordan meldingsstrukturen bør se ut kan det lønne seg å kikke litt i kapittel 3.1 "Transmit buffers" i MCP2515-databladet. Der står det at en CAN-melding må inneholde id (TXBnSIDH- og TXBnSIDL-registrene) og lengde (TXBnDLC). Hvis man skal sende noe data i meldingen må dataen befinne seg i TXBnDm-registrene. I alle registrene nevnt her er n=buffernummer. I TXBnDm er m=databytenummer (0-7).

Videre kan man ta en tur ned i "Table 11-1" i kapittel 11 i MCP2515-databladet. Der finner man adressene til alle registrene til alle bufrene. For sendebuffer nr. 0 blir det som følger:

~~~c
// Send-buffer 0
#define MCP_TXB0SIDH 0x31 //0b0011 0001
#define MCP_TXB0SIDL 0x32 //0b0011 0010
#define MCP_TXB0EID8 0x33
#define MCP_TXB0EID0 0x34
#define MCP_TXB0DLC 0x35 //0b0011 0101
#define MCP_TXB0D0 0x36 //0b0011 0110
#define MCP_TXB0D1 0x37
#define MCP_TXB0D2 0x38
#define MCP_TXB0D3 0x39
#define MCP_TXB0D4 0x3A
#define MCP_TXB0D5 0x3B
#define MCP_TXB0D6 0x3C
#define MCP_TXB0D7 0x3D
~~~

`MCP_TXB0EID8` og `MCP_TXB0EID0` er registre som trengs om man skal bruke ekstra laaaang id. Det trenger antakelig ikke dere å bry dere med.

`MCP_TXB0D1`-`MCP_TXB0D7` er antakelig ikke nødvendig å ha med. Det er fordi dere sannsynligvis finner ut at det er lurt å skrive til dataregistrene ved hjelp av ei løkke som tar utgangspunkt i databyte nr. 0 (altså skriver til `MCP_TXB0D0 + i`).

Vi endte opp med å definere følgende registre:

~~~c
// Send-buffer 0
#define MCP_TXB0SIDH 0x31
#define MCP_TXB0SIDL 0x32
#define MCP_TXB0DLC 0x35
#define MCP_TXB0D0 0x36

//Send-buffer 1
#define MCP_TXB1SIDH 0x41
#define MCP_TXB1SIDL 0x42
#define MCP_TXB1DLC 0x45
#define MCP_TXB1D0 0x46

//Send-buffer 2
#define MCP_TXB2SIDH 0x51
#define MCP_TXB2SIDL 0x52
#define MCP_TXB2DLC 0x55
#define MCP_TXB2D0 0x56

// Motta-buffer 0
#define MCP_RXB0SIDH 0x61
#define MCP_RXB0SIDL 0x62
#define MCP_RXB0DLC 0x65
#define MCP_RXB0D0 0x66

// Motta-buffer 1
#define MCP_RXB1SIDH 0x71
#define MCP_RXB1SIDL 0x72
#define MCP_RXB1DLC 0x75
#define MCP_RXB1D0 0x76
~~~

Neste steg på veien mot et lykkelig liv er å lese om de forskjellige registrene og finne ut hva i alle dager man skal bruke dem til. I kapittel 3 "Message transmission" i MCP2515-databladet finnes en hel del registerforklaringer. Det kan lønne seg å se på alle, men det mest nødvendige i denne omgang er "REGISTER 3-3: TXBnSIDH", "REGISTER 3-4: TXBnSIDL", "REGISTER 3-7: TXBnDLC" og "REGISTER 3-8: TXBnDm".

Når du kikker på SIDH- og SIDL-registrene ser du at meldings-IDen ("standard identifier bits") strekker seg over hele SIDH i tillegg til de tre øverste bitene i SIDL. Melding-ID består altså av 8+3=11 bit. Om du skal sende en melding med id 1 (0b0000 0000 001) så må SID0-bitet i SIDL-registeret være 1 og resten av bitene SID1-SID10 være 0.

Om man ser på de tre andre bitene i SIDL som vi ikke skal bry oss med (altså `EXIDE`, `EID17` og `EID16`) så ser man at EID17-16 er "extended identifier bits". Jeg er frekk nok til å anta at 11 identifiseringsbit er nok, og konkluderer med at vi kan droppe extended identifier bits. Ser du på `EXIDE`-bitet så finner du ut at det er dette som styrer hvorvidt man skal skru på extended identifier bits. Så lenge `EXIDE`=0 så er extended identifier bits skrudd av. Med andre ord: når man skal skrive SID0-SID3-bitene til SIDL-registeret er det ikke så farlig hva de andre bitene blir, så lenge `EXIDE` alltid settes til 0.

`TXBnDLC` skal ganske enkelt inneholde lengden på dataen i meldingen. `RTR`-bitet kan være 0 fordi vi skal sende en "data frame".

`TXBnDm` skal inneholde et databyte.

I kapittel 4 "Message reception" finner man "REGISTER 4-4: RXBnSIDH", "REGISTER 4-5: RXBnSIDL", "REGISTER 4-8: RXBnDLC" og "REGISTER 4-9: RXBnDM". Disse funker på akkurat samme måte som registrene i sendebufrene.

Vi lagde en meldingsstruktur ved hjelp av en struct. Meldingene du skal sende ut på CAN trenger en id (11 bit), lengde (0-8) og data (0-8 byte).

IDen er egentlig ikke så farlig akkurat nå. Mens du tester ønsker du antakelig uansett å ta imot alle meldinger uavhengig av id.

Data-arrayet i structen blir initialisert som et char-array med 8 chars. Dermed kan vi enkelt sende tekststrenger over CAN, men det kan også brukes til å sende for eksempel en enkelt 8-bit int (uint8_t).

~~~c
typedef struct Message {
	unsigned int id;
	uint8_t length;
	char data[8];
} message_t, *message_ptr;
// message_t blir et alias for "struct Message". message_ptr blir et alias for message_t* og struct* Message.
~~~

Når man sender en melding må man legge id inn i SIDH- og SIDL-registrene, datalengde inn i DLC-registeret og eventuelt data inn i D0-D7-registrene.

Vi valgte å lage en funksjon som tar inn en peker til en meldingsstruct. Deretter henter vi id, lengde og data ut fra structen og legger informasjonen i riktige registre. Til slutt sender vi en request to send-instruksjon som gjør at MCP2515 legger meldingen ut på CAN-bussen.

~~~c
void can_send(message_ptr message) {
	// Alt her foregår med buffer 0

	// Id. TXBnSIDH og TXBnSIDL
	mcp_write(MCP_TXB0SIDH, message->id / 8); // De åtte høyeste bitene i iden.
	mcp_write(MCP_TXB0SIDL, (message->id % 8) << 5); // De tre laveste bitene i iden.

	// Lengde. TXBnDLC
	mcp_write(MCP_TXB0DLC, message->length);

	// Melding. TXBnDm
	for (int i = 0; i < message->length; i++) {
		mcp_write(MCP_TXB0D0 + i, message->data[i]);
	}

	// Request to send
	mcp_request_to_send(0);
~~~

Mottaksfunksjonen vår er egentlig bare en omvendt-ish sendefunksjon. Her lager vi først en meldingsstruct som vi skal lagre dataen vi mottar i. Deretter henter vi id fra SIDH og SIDL og legger det inn i id-feltet i meldingsstructen. Lengde og data leses også fra bufferen og legges inn i meldingsstructen, og så returneres meldingsstructen med all informasjonen i.

~~~c
message_t can_receive() {
	// Alt her foregår med buffer 0
	message_t message = {};

	// Id. RXBnSIDH og RXBnSIDL
	uint8_t id_low = mcp_read(MCP_RXB0SIDL)/0b100000;
	uint8_t id_high = mcp_read(MCP_RXB0SIDH);
	message.id = id_high * 0b1000 + id_low;

	// Lengde. RXBnDLC
	message.length = mcp_read(MCP_RXB0DLC);

	// Melding. RXBnDM
	for (int i = 0; i < message.length; i++) {
		message.data[i] = mcp_read(MCP_RXB0D0 + i);
	}

	return message;
}
~~~

#### L5 Oppgave 6

Nå bør det være mulig å teste litt CAN-greier. Fortsatt i loopbackmodus. For å teste må du sette MCP2515 i MODE_LOOPBACK. Deretter sende en melding, og så motta meldingen. Og så gjerne printe den mottatte meldingen via seriell/uart. Som vanlig er det mulig å løse dette med interrupts, men vi valgte å teste uten interruptgreier.

*main.c*:
~~~c
int main(void){
	uart_init(UBRR);
	can_init(); // Denne initierer mcp, som initierer spi.
	mcp_set_mode(MODE_LOOPBACK);

	// Sender melding
	message_t message = {
		1, // Id
		6, // Lengde på dataen
		"heiiii" // Data. Maks åtte byte
	};
	can_send(&message); // Sender melding

	// Nå er meldingen sendt. Fordi vi er i loopbackmodus blir meldingen umiddelbart "mottatt" ac MCP2515.

	// Mottar melding
	message_t receive = can_receive();
	printf("Heisann sveisann, vi har fått ei melding.\r\n");
	printf("Id: %d \r\n", receive.id);
	printf("Lengde: %d \r\n", receive.length);
	printf("Melding: %s \r\n\r\n", receive.data);

	return 0;
}
~~~
