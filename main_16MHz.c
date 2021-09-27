//ICC-AVR application builder : 13/11/2014 11:40:07
// Target : T2313
// Crystal: 16.0000Mhz

#include <iot2313v.h>
#include <macros.h>

//************************** DECLARACIONES *************************************
void Config(void);
void MideF(char sensor, char color);
void EnviaHex(unsigned int valor);
void delayus(unsigned int usec);

unsigned int conteo;
char banConteo;
char caracter;
unsigned char RGBW[8][4]; //resultados




//************************ PROGRAMA PRINCIPAL **********************************
void main(void)
{
 int sensor,color;
 Config();
 EnviaHex(200);
 
 
 while(1)
 {
  //Esperar comando del serial
  while((UCSRA&0x80)!=0x80){}; //bandera RXC
  caracter=UDR;
  if(caracter=='$') //Si comando de inicio...
  {
   UDR='$'; //comenzar transmision de la cadena
	
   MideF(0,0); //rojo
   while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
   EnviaHex(conteo);//RGBW[0]; //enviar valor Rojo
   MideF(0,1); //verde
   while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
   EnviaHex(conteo);//RGBW[0]; //enviar valor verde
   MideF(0,2); //azul
   while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
   EnviaHex(conteo);//RGBW[0]; //enviar valor azul
   MideF(0,3); //blanco
   while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
   EnviaHex(conteo);//RGBW[0]; //enviar valor blanco
   
   delayus(450); //esperar 500us
   UDR='$'; //finalizar transmision de la cadena
   while((UCSRA&0x40)!=0x40){}; //bandera TXC 1.044ms aprox
   UCSRA|=0x40; //apagar bandera TXC
   
   
   
   /*
   for(sensor=0;sensor<6;sensor++)
   {
	 for(color=0;color<4;color++)
	 {
	  MideF(sensor,color);
  	  while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
	  RGBW[sensor][color]=conteo;
	  EnviaHex(conteo);
	 }
   }
   */
  }
 }


}



//******************************* FUNCIONES ************************************
//funcion que mide la frecuencia del pin CH
void MideF(char sensor, char color)
{
 //sleccionar color
 //S3=PD4 S2=PD6   Color
 //  0      0      Rojo
 //  0      1      Blanco
 //  1      0      Azul
 //  1      1      Verde
 switch(color)
 {
  case 0: PORTD &= ~0x10; //PD4=0 rojo
  	   	  PORTD &= ~0x40; //PD6=0
		  break;
  case 1: PORTD |=  0x10; //PD4=1 verde
  	   	  PORTD |=  0x40; //PD6=1
		  break;
  case 2: PORTD |=  0x10; //PD4=1 azul
  	   	  PORTD &= ~0x40; //PD6=0
	      break;
  case 3: PORTD &= ~0x10; //PD4=0 blanco
  	   	  PORTD |=  0x40; //PD6=1
		  break;
  default: break;
 }
 
 //Seleccionar sensor
 switch(sensor)
 {
  case 0:   PORTB = 0x00;//0x01; //sensor 0 CHECAR
			break;
  case 1:   PORTB = 0x02; //sensor 1
			break;
  case 2:   PORTB = 0x04; //sensor 2
			break;
  case 3:   PORTB = 0x08; //sensor 3
			break;
  case 4:   PORTB = 0x10; //sensor 4
			break;
  case 5:   PORTB = 0x20; //sensor 5
			break;			
  case 6:   PORTB = 0x40; //sensor 6
			break;
  case 7:   PORTB = 0x80; //sensor 7
			break;			
  default: break;
 }
  
 //inicia la medicion
 banConteo=0;  
 //iniciar valor del conteo timer 1 en 0s
 TCNT1H = 0x00;
 TCNT1L = 0x00;
 //iniciar Timer 0, ventana de tiempo 500us a 16Mhz
 TCNT0 = 0x83;  //set count
 TCCR0B = 0x03; //start timer CLK/64 = 2KHz, a 16MHz
 //comenzar conteo Timer1
 TCCR1B = 0x07; //start Timer T1 entrada de pulsos
 SEI(); //re-enable interrupts
}

//Convierte char a hex y lo envia por el puerto serie, tarda 2.6ms aprox
void EnviaHex(unsigned int valor)
{
 unsigned int VHL[2],aux;
 if(valor>255) valor=255;
 aux=valor;
 //Conversion de binario a hex
 aux=aux/16;  		 //separar parte entera
 VHL[1]=aux;     		 //parte alta
 VHL[0]=valor-aux*16; 	 //Parte baja
 
 //Conversion hex a char
 for(aux=0; aux<2; aux++)
 {
  if(VHL[aux]<=9) VHL[aux]=VHL[aux]+48; //menores a 9
  else VHL[aux]=VHL[aux]+55; //10 a 15
 }
 //Transmision de datos
 UDR=VHL[1]; //Parte alta
 while((UCSRA&0x40)!=0x40){}; //bandera TXC 1.044ms aprox
 UCSRA|=0x40; //apagar bandera TXC
 delayus(450); //esperar 500us 
 UDR=VHL[0]; //Parte baja
 while((UCSRA&0x40)!=0x40){}; //bandera TXC 1.044ms aprox
 UCSRA|=0x40; //apagar bandera TXC
 //aqui no conviene esperar, tanto
 //el tiempo de adquisicion del siguiente dato es de 1/2ms 
}


void Config(void)
{
 //stop errant interrupts until set up
 CLI(); //disable all interrupts
 //configurar puertos
 PORTB = 0xFF;
 DDRB  = 0xFF; 
 PORTD = 0x00;
 DDRD|=(0x10 | 0x40| 0x02); //PD4, PD6 y PD1 en salida TX
  
 //Puerto Serie sin interrupciones 9600 baudios a 16MHz
 UCSRB = 0x00; //disable while setting baud rate
 UBRRH = 0x00; //set baud rate upper
 UBRRL = 0x67; //set baud rate lower 9600 baud 
 UCSRA = 0x00; //Flags: bit7 RXC, bit6 TXC, solo limpiar TXC
 UCSRC = 0x06;
 UCSRB = 0x18; //enable
 
 //configurar timer 0
 //TIMER0 initialize - prescale:64
// WGM: Normal
// desired value: 2KHz
// actual value:  2,000KHz (0,0%)
 TCCR0B = 0x00; //stop timer 
 TCCR0A = 0x00; //start timer ok, no mover
 OCR0A = 0x7D;  //set count ok, no mover
 OCR0B = 0x7D;  //set count ok, no mover
 //TCNT0 = 0x83;  //set count
 //TCCR0B = 0x03; //start timer CLK/64 = 2KHz, a 16MHz
 
 //configurar timer 1
 //TIMER1 initialize - prescale:Rising edge as a counter
 // WGM: 0) Normal, TOP=0xFFFF
 // desired value: 1Hz
 // actual value: Out of range
 TCCR1B = 0x00; //stop timer1
 TCCR1A = 0x00;
 // TCCR1B = 0x07; //start Timer T1 entrada de pulsos
 
 //interrupciones timers
 MCUCR = 0x00;
 GIMSK = 0x00;
 TIMSK = 0x81; //timer1 ovf y timer0 compA
 //SEI(); //re-enable interrupts
 //all peripherals are now initialized
}

void delayus(unsigned int usec)
{
 unsigned int i;
 usec=usec*3;
 for(i=0; i<(usec);i++){}; 
}

//****************************** INTERRUPCIONES ********************************
#pragma interrupt_handler timer1_ovf_isr: 6  //iv_TIM1_OVF
void timer1_ovf_isr(void)
{
 TCCR1B = 0x00; //stop timer
 banConteo=2;   //conteo de frecuencia sobrepasa la capacidad del timer 1
}


#pragma interrupt_handler timer0_compa_isr:14 //iv_TIM0_COM 
void timer0_compa_isr(void)
{
 //termino el tiempo de comparacion
 TCCR1B = 0x00; //stop timer1
 TCCR0B = 0x00; //stop timer0
 
 if(banConteo==0) //no hay overflow
 {
  conteo=TCNT1;
 }
 else //overflow: frecia mayor a la capacidad de timer1
 {
  conteo=0xFFFF;
 }
 banConteo=1;
 CLI();
}

