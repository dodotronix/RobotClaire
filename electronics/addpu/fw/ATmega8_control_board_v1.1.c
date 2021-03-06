/*----------------------------------------------------------------------------------------------------*/
 /* ATmega8_control_board.c
 *
 * vytvoreno: 16.11.2014 20:57
 * Autor: Petr Pacner
 *
 *  program plni fce SLAVE na sbernici I2C tz. u mikrokontroleru ATMEL- TWI (Two Wire Interface)
 *  fce - odpovida na pozadavky od mastera
 *  lze jak zapisovat, tak i posilat vice bajtu najednou
 *
 *  popis:
 *  start bit = bit ktery poskytne SLAVE zarizeni informace o prenosu Read nebo Write
 *  nastavi se v pripade ze SCL ma log1 a SDA klesne z log1 do log0
 *  stop bit = ukoncuje komunikaci
 *  nastavi se v pripade ze SCL ma log1 a SDA stopne z log0 na log1
 *  potvrzovaci ACK - v pripade uspesneho prenosu log0 v opacnem pripade log1
 *  data se prenaseji jen kdyz je SCL na log0
 *  kontrolky desky jsou pripojeni na PORTB a pin B0; B1; B2
 */ 


#include <avr/io.h>
#include <stdint.h> 
#include <avr/interrupt.h>
#include <stdio.h>

  /* Tabulka adres registru
   * IR_ref_senzor0 == 0x00 // cislo registru se odviji od cisla pinu u A/D prevodniku
   * IR_ref_senzor1 == 0x01
   * IR_ref_senzor2 == 0x02
   * Potenciometr0 == 0x03
   *
   * Enkoder0 == 0xA0
   * Enkoder1 == 0xB0
  */

//promenna Enkoderu0
volatile long int EnkValue0 = 0; // 32-bit promenna pro zaznamenavani hodnot z enkoderu

//promenna Enkoderu1
volatile long int EnkValue1 = 0; // 32-bit promenna pro zaznamenavani hodnot z enkoderu

uint32_t Value;
unsigned char cislo;
unsigned char pocet;
unsigned char vysledek;

//rozdeli cislo na jednotlive bajty a vrati bajt podle promenne poradi
unsigned char SplitNumber (long int y, unsigned char poradi)
{

  if (poradi == 0){
    vysledek = (0xFF & y);
  }

  else if (poradi == 1){
    vysledek = ((y>>8) & 0xFF);
  }

  else if (poradi == 2){
    vysledek = ((y>>16) & 0xFF);
  }
  else if (poradi == 3){
    vysledek = ((y>>24) & 0xFF);
  }
  return vysledek;
}

/* A/D prevodnik                                                                                       */
/*-----------------------------------------------------------------------------------------------------*/

//nastavi fci A/D prevodniku
void ADinitialize ()
{
   ADCSRA |= (1 << ADEN) | (1 <<ADPS0) | (1<<ADPS1);  // povoli A/D prevodnik a nastavi  prescaler na freq/8 
}

// A/D converter - mereni hodnot z Portu C (piny: 0, 1, 2, 3)
// dostane za parametr kanal, na kterem chceme merit
// fce vraci namerenou hodnotu z prevodu

int ADmeasure (unsigned char channel)
{
    ADMUX = 0x40 | channel; // zapnout referencni (interni) napeti [2,56V]
    // (1 << 7) tento bitovy posun znamena, ze chceme posunout jednicku o sedm mist do leva
    ADCSRA |= (1 << ADSC) | (1 <<ADPS0) | (1<<ADPS1);  // zacne merit
    while ((ADCSRA & 0x10)==0);// cekaci smycka - DOKONCENO ? jednotlive mereni

    ADCSRA |= 0x10;
    return ADCW; // vraci namerenou hodnotu z registru ADCH a ADCL
                // ADCH registr musi byt precten jako prvni, aby mohli byt aktualizovany data v nich
}


/* sledovani enkoderu1 a 2                                                                            */
/*----------------------------------------------------------------------------------------------------*/
// inkrementalni rotacni enkodery - enkoder slozeny ze dvou IR reflexnich senzoru, ktere jsou o 90 stupnu pootocny vuci sobe
// dulezite pro rizeni motoru - motory jsou nestejne vyrobene (zalezi na presnosti kadeho motoru), proto se kazdy motor
// bude at uz pomaleji ci rychleji otacet; vysledek = nechtene  zhybani robota
// elektromotor - toci se na kazdou stranu jinak rychle

/* INTEGROVANE ENKODERY POLOLU
 * 64 kmitu za otacku enkoderu, pocet otacek hridele vystupujici z prevodovky je 1216 kmitu
 * pri maximalnich moznych otackach (500 RPM) bude behem sekundy zaznamenano 10 133,33 tiku
 * program - sleduje dva enkodery na Portu D
*/

/* POPIS FCE PROGRAME PRO CTENI ENKODERU
 * enkodery jsou sledovany jen kdyz je na vstupnich pinech log1 -> je tedy nutne pouzit AVR fci interrupt (preruseni)
 * PD2 = INT0 a PD3 = INT1 interupt piny
 * Enkoder_Motor0 - A == Pin PD2; B == Pin PD4
 * Enkoder_Motor1 - A == Pin PD3; B == Pin PD5
 * na pinech PD2 a PD3 je zapnut interrupt -> jakmile se zmeni hodnota pinu PD2 nebo PD3 na log1  program zacne piny sledovat
 * -> pricte nebo odecte jednotku k promenne EnkValue0 nebo 1  podle toho, jestli se robot pohybuje dopredu nebo dozadu
 
*  tzn. kdyz ma senzor A log1 a B ma log0 odecte se jednicka v pripade log1 se jednicka pricte
 *  rychlost pohybu pocita RPi
 *
*/

/* inticializace interruptu                                                    */
/*-----------------------------------------------------------------------------*/

void inicializateInterrupt ()
  {
  /* interupty a reset vektor maji kazdy programove vektory v programovem pametovem ulozisti
   * kazdy interrupt je samostatny pro pouziti musi byt spolecne nastaven na log1  globalni interrupt bit ve status registru 
   * tyto interrupty mohou byt automaticky zakazany, kdyz bude nastaven alespon jeden z fuses bitu BLB02 nebo BLB12
   * nejnizsi hodnoty adres jsou defaultne nastaveny jako reset a prerusovaci vektory
   * kazdy interupt ma jinou prioritu; vektory definuji prioritu interruptu (reset-nejvyssi priorita -> INT0 -> INT1...)
   * nastavenim IVSEL bitu v registru GICR -> presun interrupt vektoru na zacatek flashove sekce
   * presun je mozny take pro RESET -> nastavenim BOOTRST FUSE
   * makro IRS () nastavi Global Interrupts Enable (1-bit) v SREG; a brani behu interruptu, kdyz na log0
   * GLobal Interrupt Enable je smazan po objeveni se interruptu
   * pro znovu aktivovani intrruptu zapsat jednicku do GIE, nebo se po interruptu vrati instrukce RETI
   * RETI - (return from interrupt); adresa na vraceni se nacte ze zasobniku a global interrupt flag je nastaven na log1

   * dva typy preruseni:
   * 1) interrupt je vybuzen udalosti, ktera nastavi interrupt flag
   *    - kdyz se zacne vykonavat interupt vymaze se odpovidajici interrupt flag
   *    - objevi-li se interrupt a GIE neni nastaven -> atmega pocka nez az bude nastaven -> provede interrupt podle priority
   *    - je-li detekovan interrupt, ktery neni povolen -> nastavi se a ulozi interrupt flag -> vykona interrupt, az bude 
          povolen

   * 2) interrupt je spusten po dobu jeho trvani - kdyz je drive drive ukoncen nez spusten -> zadna akce
   *    - vzdy navrat k main programu -> vykonani jedne instrukce -> dalsi interrupt bude vykonan
   *    - zkontrolovat - Status Registr nesmi byt automaticky ulozen at pred nebo po interuptu - nutno vyresit softwarove
   *    - CLI - instrukce na okamzite zakazani interruptu -> po ni nebude proveden zadny interrupt

   * cas odpovedi preruseni - po 4 cyklech je interrupt vyrizen - return trva stejnou dobu
  */

  PORTB |= 0x04;
  MCUCR |= (1<<ISC00) | (1<<ISC01) | (1<<ISC10) | (1<< ISC11); // interrupt se zavola, jakmile hodnota na INT0 a INT1 ==logi
  // MCUCR - registr ktery nastavuje, na co bude externi interrupt reagovat viz. dokumentace atmega8
  // ISC0 == INT0
  // ISC1 == INT1
  GICR |= (1<<INT1) | (1<<INT0); // povoli externi interrupt piny INT0(PD2) a INT1(PD3)
  sei (); // zapne fci interrupt
}


/* NASTAVENI I/O REGISTRU
* 1) DDRx (x == cislo portu) - nastavuje smer, tedy zda se ma pin nastavit jako input (log0) nebo output(log1)
* 2) PORTx - tento registr dokaze vypinat a zapinat piny nastavene jako output (log1)
* v pripade stavu input (log0) aktivuje pull up resistory (log1) vypnuto (log0)
* 3) PINx - registre, ktery registruje vstupni hodnoty 
*/

// pokud je vystupni cislo z enkoderu zaporne, pak je rozsah hodnot od 0-255 (jeden bajt),
// jestli-ze zaporne cislo prekroci hodnotu -255, pak pujde pocitani znovu od nuly do 255.

// makro ISR()  funguje jako ovladac interuptu s vektorem, ktery je v zavorce jako parametr fce
// fce pro sledovani enkoderu 0
ISR (INT0_vect) // vector INT0 (PD2)
{
      if ((PIND &= 0x10) == 0) // vymaskuje pin 4 (vodic B [bily]); je na vstupu log0?
      {
        EnkValue0 -= 1;
      }

      else // na vstupu je log1
      {
        EnkValue0 += 1;
      }
}


// fce pro sledovani  enkoderu 1
ISR (INT1_vect) //vektor INT1 (PD3)
{
      if ((PIND &= 0x20) == 0) // vymaskuje pin 5 (vodic B [bily]); je na vstupu log0?
      {
        EnkValue1 -= 1;
      }

      else // na vstupu je log1
      {
        EnkValue1 += 1;
      }
}


/* I2C_SLAVE_DEVICE                                                                                    */
/*-----------------------------------------------------------------------------------------------------*/

void I2Cinicializace ()
{
  TWAR = 0x64; // adresa ATmegy8
  TWCR = (1<<TWEN) | (1<<TWEA); //TWEN - zapne TWI rozhrani
  //TWEA - zapne potvrzovaci zpravu, kdyz bude MASTER volat danou adresu ATmegy
  //Bit slouzi take na potvrzovaci zpravu na general call
  // SLA+R/SLA+W - bit ktery oznacuje, zda se ma zapisovat nebo cist
  //general call - specialni adresa, ktera slouzi k posilani dat nekolika zarizenim najednou
}


/* Fce I2CReceiveTransmitt - vyckava neustale na MASTERuv signal (na adresu)
 * Raspberry Pi pouziva WiringPi knihovny - zde muzeme cist pouze dva bajty najednou
 * Raspberry pozada o hodnotu, ktera je v jednom z vyse uvedenych registru
*/
void I2CReceiveTransmitt ()
{// knihovna WiringPi neumi posilat bajty zasebou s repeated startem ale jen oddelene, proto jsou jednotlive cteni oddeleny NACK bitem
  while(1)
  { 
        // (!) logicka negace; kdyz je while (0) program pokracuje dale
        while(!(TWCR & (1<<TWINT)));  // cekaci smycka; je TWINT == 1? (nenulove cislo)
        PORTB |= 0x02;
       
        if((TWSR & 0xF8)==0xA8) // ATmega prijala SLA+R od MASTERa a vratila ACK
        {
            if(pocet == 0){
              pocet = 4;
              Value = EnkValue1;
            }
            TWDR = SplitNumber (Value,(4-pocet));
            --pocet;
	          TWCR &=~(1<<TWEA); // odesle NACK 
        }

         else if((TWSR & 0xF8)==0xC0) // bajt v TWDR byl odeslan, NACK byl MASTERem vracen
        {
	  PORTB &= 0xFd;
          TWCR |= (1<<TWINT) | (1<<TWEA); //vynuluje se bit TWINT (zapsanim log1); nastavi se TWEA na log1 (zapsanim log1)
          // ATmega se vrati do pripraveneho stavu pro komunikaci s MASTERem; bude naslouchat zda MASTER nevola jeji adresu
        }

        else if ((TWSR & 0xF8) == 0x60) // ATmega prijala SLA+W od MASTERa a vrati ACK         
        {
	  TWCR |= (1<<TWINT) | (1<<TWEA); // vrat ACK
        }

	else if((TWSR & 0xF8) == 0x80) //znovu adresovan -> data prijmuta od mastera (write) -> vracen ACK
        {
	          // jaky senzor se ma zmerit?
          if (TWDR == 0x00) //IR_senzor0
          {
            Value = (ADmeasure (0x00));
            // 32-bit cislo ma hodnotu -1
          }

          else if (TWDR == 0x01) //IR_senzor1
          {
            Value = (ADmeasure (0x01));
          }

           else if (TWDR == 0x02) //IR_senzor2
          {
            Value = (ADmeasure (0x02));
          }

           else if (TWDR == 0x03)
          {
            Value = (ADmeasure (0x03)); //Potenciometr
          }

           else if (TWDR == 0xB0) //ENKODER0
          {
            Value = EnkValue0;
          }

	  pocet = 4; // pocet bajtu, ze kterych je cislo slozeno?
	  TWCR |= (1<<TWINT) | (1<<TWEA); // ukonci komunikaci a vyckava dale na sbernici zda neni volana                                  
        }  
        
        else if((TWSR & 0xF8) == 0xA0) // byla prijat STOP signal nebo byl uskutecnen opakovany START (prenos byl prerusen)
        {                                  
          TWCR |= (1<<TWINT) | (1<<TWEA); //vynuluje se bit TWINT (zapsanim log1); nastavi se TWEA na log1 (zapsanim log1)
          // ATmega se vrati do pripraveneho stavu pro komunikaci s MASTERem; bude naslouchat zda MASTER nevola jeji adresu
        } 

        else if ((TWSR & 0xF8) == 0xF8) // zarizeni neni MASTERem vyzadano; TWINT je 0, zadna nova zprava v TWSR
        {
	  //nenastane
          // (jiny stav)
        }
     
        else if ((TWSR & 0xF8) == 0x00) // ERROR: vyskytl se STOP/START signal behem prenosu, prenos nedokoncen
       {
	  //nenastane
          TWCR |= (1<<TWINT) | (1<<TWSTO); // vynuluje se bit TWINT (zapsanim log1); nastavi se TWSTO na log0 (zapsanim log1)
          // ATmega se vrati do pripraveneho stavu pro komunikaci s MASTERem; bude naslouchat zda MASTER nevola jeji adresu
       }

  }
}

/* program                                                                                             */
/*_____________________________________________________________________________________________________*/

int main ()
{
     DDRB = 0x07; // inicializace vystupniho pinu PB5
     PORTB |= 0x01; // rozsvit vsechny LED (poradi: zluta, modra, cervena)

    ADinitialize ();
    I2Cinicializace ();
    inicializateInterrupt ();
    I2CReceiveTransmitt ();

return 0;
}
