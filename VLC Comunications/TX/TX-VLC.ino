//*********************************************************************
//#///////////////////!!! Declarare bibliotecilor !!!\\\\\\\\\\\\\\\\\\# 
#include <TimerOne.h>
#include <util/atomic.h>
#define SYMBOL_PERIOD 500 // schimbarea perioadei definte pe simbol  
#define WORD_LENGTH 10 // Declararea lungimii maxime a unui pachet, respectiv 10 biti format din start, continut, stop 
#define SYNC_SYMBOL 0xD5 // Simbolul de sincronizare care este trimis inainte ca comunicatia sa inceapa 
#define ETX 0x03 //simbol de sfarsit a cadrului
#define STX 0x02 //simbol de start a cadrului
//Definirea pinilor de comunicatie. 
//Definirea pinului 13 si a LED-ului conectat la pinul D13
#define OUT_LED() DDRB |= (1 << 5);
#define SET_LED() PORTB |= (1 << 5)
#define CLR_LED() PORTB &= ~(1 << 5)
//Definirea pinului 2 si a LED-ului conectat la pinul D2
/*#define OUT_LED() DDRD |= ((1 << 2))
#define SET_LED() PORTD |= ((1 << 2))
#define CLR_LED() PORTD &= ~((1 << 2))
*/
//Definirea pinilor 2, 3,4 si a LED-urilor conectate la D2, D3, D4
/*#define OUT_LED() DDRD |= ((1 << 2) | (1 << 3) | (1 << 4))
#define SET_LED() PORTD |= ((1 << 2) | (1 << 3) | (1 << 4))
#define CLR_LED() PORTD &= ~((1 << 2) | (1 << 3) | (1 << 4))
*/
unsigned char frame_buffer [38] ; //buffer pentru cadre
char frame_index = -1; // index in cadru
char frame_size = -1  ; // marimea cadrului care va fi trimis
//starea valorilor  pentru codificatorul Manchester  
unsigned char bit_counter = 0 ;
unsigned short data_word = 0 ;  //8 bit  de date + 1 bit de start + 1 bit de stop
unsigned char half_bit = 0 ;
unsigned long int manchester_data ;
void to_manchester(unsigned char data, unsigned long int * data_manchester){
  unsigned int i ;
 (*data_manchester) = 0x02 ; // simbolul care semnifica bitul de STOP 
 (*data_manchester) = (*data_manchester) << 2 ;
  for(i = 0 ; i < 8; i ++){
    if(data & 0x80) (*data_manchester) |=  0x02  ; 
    else (*data_manchester) |= 0x01 ;
    (*data_manchester) = (*data_manchester) << 2 ;
    data = data << 1 ; // se merge la urmatorul bit 
  }
  (*data_manchester) |= 0x01 ; //bitul de START  
}
  //intreruperea emitorului 
  void emit_half_bit(){
     if(manchester_data & 0x01){
       SET_LED();
     }else{
       CLR_LED();
     }
     bit_counter -- ;
     manchester_data = (manchester_data >> 1);
     if(bit_counter == 0){   
        // aici se verifica daca mai sunt biti pentru a fi trimisi in cadrul unui cadru 
        manchester_data = 0xAAAAAAAA ; // se trimite in continuu acelasi mesaj daca nu este un mesaj nou 
        if(frame_index >= 0 ){
          if(frame_index < frame_size){
            to_manchester(frame_buffer[frame_index], &manchester_data);
            frame_index ++ ;
          }else{
            frame_index = -1 ;
            frame_size = -1 ;}}
        bit_counter = WORD_LENGTH * 2 ;}}
void init_frame(unsigned char * frame){
  memset(frame, 0xAA, 3);
  frame[3] = SYNC_SYMBOL ;
  frame[4] = STX;
  frame_index = -1 ;
  frame_size = -1 ;}
int create_frame(char * data, int data_size, unsigned char * frame){
  memcpy(&(frame[5]), data, data_size);
  frame[5+data_size] = ETX;
  return 1 ;}
int write(char * data, int data_size){
  if(frame_index >=  0) return -1 ;
  if(data_size > 32) return -1 ;
  create_frame(data, data_size,frame_buffer);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    frame_index = 0 ;
    frame_size = data_size + 6 ;}
  return 0 ;}
int transmitter_available(){
  if(frame_index >=  0) return 0 ;
  return 1 ; }
void init_emitter(){
  manchester_data = 0xFFFFFFFF ;
  bit_counter = WORD_LENGTH * 2 ;}
//#///////////////////!!! Finalizarea declararii !!!\\\\\\\\\\\\\\\\\\# 
//*********************************************************************
//*********************************************************************
//#///////////////////!!! Initializare Setup !!!\\\\\\\\\\\\\\\\\\# 
void setup() {
  Serial.begin(115200);  // initializarea interfetei de comunicare seriale cu calculatorul cu viteza serialei de 115200 Bps:
  OUT_LED();
  init_frame(frame_buffer);
  init_emitter();
  Timer1.initialize(SYMBOL_PERIOD); //declararea intreruperilor pentru rutina
  Timer1.attachInterrupt(emit_half_bit); }
// Declararea mesajelor care pot fi trimise prin lumina sub forma de mesaj
char * msg = "TEST" ;
char * msg1 = "Rosu" ;
char * msg2 = "Galben" ;
char * msg3 = "Verde" ;
char com_buffer [32] ;
char com_buffer_nb_bytes = 0 ;
//#///////////////////!!! Finalizarea Setup !!!\\\\\\\\\\\\\\\\\\# 
//*********************************************************************
//*********************************************************************
//#///////////////////!!! Initializare Loop !!!\\\\\\\\\\\\\\\\\\# 
void loop() {
  send_data(msg1);
  send_data(msg2);
  send_data(msg3);
  }
//#///////////////////!!! Finalizarea Loop !!!\\\\\\\\\\\\\\\\\\# 
//*********************************************************************
//*********************************************************************
//#///////////////////!!! Initializare Functie de trimitere !!!\\\\\\\\\\\\\\\\\\# 
int send_data(short int msg){
  #ifdef TRANSMIT_SERIAL
  if(Serial.available() && transmitter_available()){ //constructia cadrului de date, doar daca transmitatorul este disponibil
    char c = Serial.read();
    com_buffer[com_buffer_nb_bytes] = c ;
    com_buffer_nb_bytes ++ ;
    if(com_buffer_nb_bytes >= 32 || c == '\n'){
      if(write(com_buffer, com_buffer_nb_bytes) < 0){
        Serial.println("Transmitatorul nu este pregatit");// Daca transmitatorul nu este disponibil se afiseaza acest mesaj
      }else{
        com_buffer_nb_bytes = 0 ;}}}
  delay(10);
  #else
    static int i = 0 ;
    memcpy(com_buffer, msg, 11);
    com_buffer[11] = i + '0' ;
    if(write(com_buffer, 12) < 0){
      delay(10);
    }else{
      i ++ ; 
      if(i > 9) i = 0 ;}
  #endif
  delay(600);// retinerea programului pentru ca transmitatorul sa reuseasca sa termine sesiunea de trimitere}
//#///////////////////!!! Finalizarea functie trimitere/ reluare functie loop !!!\\\\\\\\\\\\\\\\\\# 
//*********************************************************************
