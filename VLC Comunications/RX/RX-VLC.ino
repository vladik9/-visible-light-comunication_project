//*********************************************************************
//#///////////////////!!! Declarare bibliotecilor !!!\\\\\\\\\\\\\\\\\\# 
#include <TimerOne.h>
#include "receiver_types.h"// biblioteca special realizata pentru receptia Li-Fi VLC
#define INT_REF // Se recomanda de dezactivat atunci cand puterea LED-ului este redusa
enum receiver_state frame_state = IDLE ;
//Aici sunt definite proprietatile de receptie, sunt necesare de a fi setate exact ca si la emisie
#define SENSOR_PIN 0
#define SYMBOL_PERIOD 500
#define SAMPLE_PER_SYMBOL 4
#define WORD_LENGTH 10 // Declararea lungimii maxime a unui pachet, reespectiv 10 biti 
#define SYNC_SYMBOL 0xD5 // Simbolul de sincronizare care este trimis inainte ca comunicatia sa inceapa 
#define ETX 0x03 // simbol de sfarsit a cadrului 
#define STX 0x02 //simbol de start a cadrului
// variabilele generale pentru decodificarea cadrului 
char frame_buffer[38] ;
int frame_index  = -1 ;
int frame_size = -1 ;
//variabile de stare ale pragului de receptie
unsigned int signal_mean = 0 ;
unsigned long acc_sum = 0 ; //este utilizat pentru calcularea valorea medie a semnalului
unsigned int acc_counter = 0 ;
// variabila de stare a decodificatorului Manchester
long shift_reg = 0;
//Start pentru functiile  ADC de gestionare
void ADC_setup(){
  ADCSRA =  bit (ADEN);                      // Activare ADC  
  ADCSRA |= bit (ADPS0) |  bit (ADPS1) | bit (ADPS2);  
  #ifdef INT_REF
  ADMUX  =  bit (REFS0) | bit (REFS1);    
  #else
  ADMUX  =  bit (REFS0) ;   
  #endif
}
void ADC_start_conversion(int adc_pin){
  ADMUX &= ~(0x07) ; // golirea canalelor activate
  ADMUX  |= (adc_pin & 0x07) ;    // AVcc slectarea portului de intrare
  bitSet (ADCSRA, ADSC) ;
}
int ADC_read_conversion(){
 while(bit_is_set(ADCSRA, ADSC));
 return ADC ;
}
// Sfârșitul funcțiilor de gestionare ADC
#define START_SYMBOL 0x02
#define STOP_SYMBOL 0x01
#define START_STOP_MASK  ((STOP_SYMBOL << 20) | (START_SYMBOL << 18) | STOP_SYMBOL) 
#define SYNC_SYMBOL_MANCHESTER  (0x6665) // Simbolul de sincronizare, codat ca o valoare Manchester pe 16 biți, necesar pentru decodare 
inline int is_a_word(long  * manchester_word, int time_from_last_sync, unsigned int * detected_word){
        if(time_from_last_sync >= 20  || frame_state == IDLE){ // se verifica sicronizarea si daca am primit destui biți pentru a testa sincronizarea      
            if(((*manchester_word) & START_STOP_MASK) == (START_STOP_MASK)){ // testarea primei poziții
                  (*detected_word) = ((*manchester_word) >> 2) & 0xFFFF;
                  if(frame_state == IDLE){
                     if((*detected_word) == SYNC_SYMBOL_MANCHESTER) return 2 ;
                  }
                  return 1 ;
                  }else if(frame_state != IDLE && time_from_last_sync == 20){
               (*detected_word)= ((*manchester_word) >> 2) & 0xFFFF;
               return 1 ;
            }}
          return 0 ;}
inline int insert_edge( long  * manchester_word, char edge, int edge_period, int * time_from_last_sync, unsigned int * detected_word){
   int new_word = 0 ;
   int is_a_word_value = 0 ;
   int sync_word_detect = 0 ;
   if( ((*manchester_word) & 0x01) != edge ){ 
             if(edge_period > (SAMPLE_PER_SYMBOL+1)){
                unsigned char last_bit = (*manchester_word) & 0x01 ;
                (*manchester_word) = ((*manchester_word) << 1) | last_bit ; // se verifica daca semnalul a fost constant mai mult decât pe un singur simbol
                (*time_from_last_sync) += 1 ;
                is_a_word_value = is_a_word(manchester_word, (*time_from_last_sync), detected_word);
                if(is_a_word_value > 0){ 
                   new_word = 1 ;
                  (*time_from_last_sync) =  0 ;
                  if(is_a_word_value > 1) sync_word_detect = 1 ; // detecarea încadrarii și sincronizarii cuvântului în format Manchester
                }
             }
             // atribuirea volorii ultime în cuvant 
             if(edge < 0){
              (*manchester_word) = ( (*manchester_word) << 1) | 0x00 ; // semnalul se miscoreaza ca putere
             }else{
              (*manchester_word) = ( (*manchester_word) << 1) | 0x01 ; // semnalul se mareste ca putere
             }
             (*time_from_last_sync) += 1 ;
             is_a_word_value = is_a_word(manchester_word, (*time_from_last_sync), detected_word);
             if(sync_word_detect == 0 && is_a_word_value > 0){ // dacă se detecteaza cuvântul de sincronizare la o poziția anterioară, nu se mai inregistreaza alte cuvinte detectate
              new_word = 1 ;
               (*time_from_last_sync) =  0 ;
             }
          }else{
            new_word = -1 ;
          }
          return new_word ;
}
#define EDGE_THRESHOLD 4 //Defineste diferenta de tensiune intre doua probe pentru a detecta o margine in crestere / cadere. Poate fi crescuta in dependeta de mediu
int oldValue = 0 ;
int steady_count = 0 ;
int dist_last_sync = 0 ;
unsigned int detected_word = 0;
int new_word = 0;
char old_edge_val = 0 ;
void sample_signal_edge(){
  char edge_val ;
  int sensorValue  = ADC_read_conversion(); //citeste rezultatul conversiei declanșate anterior
  ADC_start_conversion(SENSOR_PIN); // se incepe o conversie pentru urmatoarea functie loop
  #ifndef DEBUG
  #ifdef DEBUG_ANALOG
  Serial.println(sensorValue, DEC);
  #endif
  #endif
  if((sensorValue - oldValue) > EDGE_THRESHOLD) edge_val = 1 ;
  else if((oldValue - sensorValue) > EDGE_THRESHOLD) edge_val = -1;
  else edge_val = 0 ;
  oldValue = sensorValue ;
  if(edge_val == 0 || edge_val == old_edge_val || (edge_val != old_edge_val && steady_count < 2)){
    if( steady_count < (4 * SAMPLE_PER_SYMBOL)){
      steady_count ++ ;
    }
  }else{  
          new_word = insert_edge(&shift_reg, edge_val, steady_count, &(dist_last_sync), &detected_word); 
          if(dist_last_sync > (8*SAMPLE_PER_SYMBOL)){ // se limiteaza dist_last_sync pentru a evita problemele de suprapunere
            dist_last_sync = 32 ; }
            steady_count = 0 ; }
        old_edge_val = edge_val ;
}

int add_byte_to_frame(char * frame_buffer, int * frame_index, int * frame_size, enum receiver_state * frame_state ,unsigned char data){
  if(data == SYNC_SYMBOL){
    (*frame_index) = 0 ;
    (*frame_size) = 0 ;
    (*frame_state) = SYNC ;
    return 0 ;
  }
  if((*frame_state) != IDLE){ // asigurarea sincornizarii
  frame_buffer[*frame_index] = data ;
  (*frame_index) ++ ;
    if(data == STX){
      (*frame_state) = START ;
       return 0 ;
    }else if(data == ETX){
      (*frame_size) = (*frame_index) ;
      (*frame_index) = -1 ;
      (*frame_state) = IDLE ;
      return 1 ;
    }else if((*frame_index) >= 38){ //se verifica daca cadrul este prea mare decat dimensiunile stabilite implicit
      (*frame_index) = -1 ;
      (*frame_size) = -1 ;
      (*frame_state) = IDLE ;
      return -1 ;
    }else{
      (*frame_state) = DATA ;
    }
    return 0 ;
  }
  return -1 ;
}
//#///////////////////!!! Finalizarea declararii !!!\\\\\\\\\\\\\\\\\\# 
//*********************************************************************
//*********************************************************************
//#///////////////////!!! Initializare Setup !!!\\\\\\\\\\\\\\\\\\# 
void setup() {
  // initializarea interfetei seriale de comunicare:
  int i; 
  Serial.begin(115200);
  Serial.println("Start of receiver program");
  ADC_setup();// initializare ADC
  ADC_start_conversion(SENSOR_PIN);
  Timer1.initialize(SYMBOL_PERIOD/SAMPLE_PER_SYMBOL); //1200 bada impartita in cadre de 4
  Timer1.attachInterrupt(sample_signal_edge);//initializare intreruperilor pentru receptie pe pinul A0
}
//#///////////////////!!! Finalizarea Setup !!!\\\\\\\\\\\\\\\\\\# 
//*********************************************************************
//*********************************************************************
//#///////////////////!!! Initializare Loop !!!\\\\\\\\\\\\\\\\\\# 
void loop() {
  int i; 
  unsigned char received_data;
  char received_data_print ;
  int nb_shift ;
  int byte_added = 0 ;
  if(new_word == 1){
    received_data = 0 ;
    for(i = 0 ; i < 16 ; i = i + 2){ //decodificare dupa Manchester
             received_data = received_data << 1 ;
             if(((detected_word >> i) & 0x03) == 0x01){
                 received_data |= 0x01 ;
             }else{
                 received_data &= ~0x01 ;
             }
    }
    received_data = received_data & 0xFF ;
    #ifdef DEBUG
      Serial.print(received_data & 0xFF, HEX);
      Serial.print(", ");
      Serial.println((char) received_data);
    #endif
    new_word = 0 ;
    if((byte_added = add_byte_to_frame(frame_buffer, &frame_index, &frame_size, &frame_state,received_data)) > 0){
      frame_buffer[frame_size-1] = '\0';
      Serial.println(&(frame_buffer[1])); }}}
//#///////////////////!!! Finalizarea Loop !!!\\\\\\\\\\\\\\\\\\# 
//*********************************************************************
