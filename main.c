

// Target : TINY84A
// Crystal: 20.0000Mhz

#include <iot84v.h>
#include <macros.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <ctype.h>
#include <stdarg.h>

//!******************************************************** CONEXIONES *****************************************************
//                                                   -----------\/-----------
//                                      5v         =| VCC          GND       |= GND
//                                      XTAL 20MHz =| PB0(XT1)     PA0(ADC0) |= NC
//                                      XTAL 20MHz =| PB1(XT2)     PA1(ADC1) |= S3
//                                      RESET      =| PB3(REST)    PA2(ADC2) |= S2
//                                      SS (SPI)   =| PB2(INT0)    PA3(T0)   |= OUT
//                                      NC         =| PA7(ADC7)    PA4(USCK) |= SCK (SPI)
//                                      DI (SPI)   =| PA6(MOSI)    PA5(MISO) |= DO  (SPI)
//                                                   ------------------------

#define TCS_OUT_DDR DDRA
#define TCS_OUT_PORT PINA
#define TCS_OUT_POS PA3

//!****************************************************** DECLARACIONES ****************************************************
void    MideF(int color, int Texp); //Texp=1 500us, Texp=2 1ms, Texp=3 1.5ms, Texp=4 2ms
void    CorrigeSensorColor(int Rr, int Gr, int Br);
void    CargaParametros(void);
void    GuardaParametros(void);
//void    EnviaHex(unsigned int valor);
uint8_t SPI_SlaveTransfer(uint8_t Dato, char llamada);
uint8_t EEPROM_read(uint16_t ucAddress);
void    EEPROM_write(uint16_t ucAddress, unsigned char ucData);
void    Config(void);
void    delayus(unsigned int usec);
void    delay_ms(uint32_t msec);

void    enviaEEPROM(void);



//!******************************************************* VARIABLES *******************************************************
//Variables

uint8_t Esclavo_ID=1, DatoSPI;
int ss=0,inicio=0;
int ovf=0;
uint8_t conteo;
uint8_t rr[4],rg[4],rb[4];
char  banConteo;
char caracter;
char Texp=1; //indice del tiempo de exposicion
char     Haciendo=0;            //parte del ciclo que se encuentra haciendo con el Master


float Ha,Sa,Va;



//Comandos de correccion de color
uint8_t BalanceNegros=1;    //Si 1: hace balance de negros
uint8_t BalanceBlancos=1;   //Si 1: hace balance de blancos
uint8_t CorregirGama=1;     //Si 1: aplica la correccion gama
uint8_t CorregirColor=0;    //Si 1: aplica matriz de correccion de color
uint8_t CorregirHSV=1;      //Si 1: aplica la correcion de color HSV
uint8_t ObtenerRGBdeHSV=0;  //Si 1: recalcula RGB de HSV
uint8_t umbralGris=120;   // umbral en HSV2RGB de grises

//Paramaetros correcion de blancos y negros
float rslm = 0.9744;
float rslb = 19.8418;
float gslm = 1.2165;
float gslb = 21.5859;
float bslm = 1.3172;
float bslb = 20.3862;

//Parametros minimos cuadrados rgb
float mcrm=1.4948;
float mcrb=-52.5315;
float mcgm=1.5475;
float mcgb=-47.1519;
float mcbm=1.4906;
float mcbb=-42.2013;

//Parametros minimos cuadrados hsv
float mchm=0.8578;
float mchb=0.0691;
float mcsm=1.5721;
float mcsb=-0.0240;
float mcvm=0.8052;
float mcvb=0.1945;

int MHSV11=255;                      // matriz de correccion HSV
int MHSV12=0;
int MHSV13=0;
int MHSV21=0;
int MHSV22=321;
int MHSV23=29;
int MHSV31=0;
int MHSV32=62;
int MHSV33=265;


uint8_t cbn[6];
float buff[18];
int buffint[9];
uint8_t storage [72], storageint[18];
size_t sizefloat;
uint8_t cr,cg,Hl,Hh;

uint8_t R,G,B;                      //Valores RGB
uint8_t S,V;                        //Valores HSB
uint16_t H;


//!***************************************************** PROGRAMA PRINCIPAL ***********************************************
void main(void)
{

    GuardaParametros();
    Config();


    //EnviaHex(200);
    //i= SPI_SlaveTransfer(0xAB); //usar con alguna interrupcion en pin
    //i= SPI_SlaveTransfer(0xF1);

//    double val,Kt=100;
//    for(i=0;i<256;i++)
//    {
//        val=i;
//        val=val/Kt;
//        val=(1-exp(val))*255;
//        Gama[i]= (unsigned char)val;
//    }
    //CargaParametros();
    //GuardaParametros();






    while(1)
    {
        if(Haciendo==0)
        {
            PORTA |= 0x06; //PA2 y PA1 apagar
            //Esperar comando del maestro
            //'C':   //Adquirir color y enviar RGB
            //'H':   //Adquirir color y enviar HSV
            //'D':   //Adquirir color y enviar RGB y HSV
            //'P':   //El maestro va a actualizar los parametros de correccion de color
            //'R':   //El maestro va a leer los parametros de correccion de color
            Haciendo=SPI_SlaveTransfer(1,0); //Esperar comando del Master
            //PORTA &= ~0x06; //PA2 y PA1 encender

        }
        else
        {
            switch(Haciendo)
            {
                case 'C':   //Adquirir color y enviar RGB
                            if(ss==1){
                                MideF(0,4); //rojo Texp=2ms
                                while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
                                SPI_SlaveTransfer(conteo,0);
                                cr=conteo;
                                MideF(1,4); //verde Texp=2ms
                                while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
                                SPI_SlaveTransfer(conteo,0);
                                cg=conteo;
                                MideF(2,4); //azul Texp=2ms
                                while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
                                SPI_SlaveTransfer(conteo,0);
                                CorrigeSensorColor(cr,cg,conteo);
                                Hh=H>>8;          //parte alta
                                Hl=H&0x00FF;   //parte baja
                                SPI_SlaveTransfer(R,0);
                                SPI_SlaveTransfer(G,0);
                                SPI_SlaveTransfer(B,0);
                                SPI_SlaveTransfer(Hl,0);
                                SPI_SlaveTransfer(Hh,0);
                                SPI_SlaveTransfer(S,0);
                                SPI_SlaveTransfer(V,0);
                                SPI_SlaveTransfer(ovf,0);


                                Haciendo=0;
                            }
                            Haciendo=0;
                            ss=0;
                            break;




                case 'H':   //Adquirir color y enviar HSV

                            break;

                case 'D':   //Adquirir color y enviar RGB y HSV

                            break;

                case 'P':   //El maestro esta actualizando los parametros de correccion de color
                            if(ss==1){
                                for(int i=1; i<7;i++){
                                    DatoSPI=SPI_SlaveTransfer(2,0);
                                    EEPROM_write(i,DatoSPI);
                                }
                                for(int i=9; i<81;i++){
                                    DatoSPI=SPI_SlaveTransfer(3,0);
                                    EEPROM_write(i,DatoSPI);
                                }
                                for(int i=200; i<218;i++){
                                    DatoSPI=SPI_SlaveTransfer(4,0);
                                    EEPROM_write(i,DatoSPI);
                                }
                                CargaParametros(); //actualiza parametro
                                Haciendo=0;
                            }
                            Haciendo=0;
                            ss=0;
                            break;

                case 'R':   //El maestro esta leyendo los parametros de correccion de color
                            if(ss==1){
                                for(int i=1; i<7;i++){
                                    DatoSPI = EEPROM_read(i);
                                    SPI_SlaveTransfer(DatoSPI,0);
                                }
                                for(int i=9; i<81;i++){
                                    DatoSPI = EEPROM_read(i);
                                    SPI_SlaveTransfer(DatoSPI,0);
                                }
                                for(int i=200; i<218;i++){
                                    DatoSPI = EEPROM_read(i);
                                    SPI_SlaveTransfer(DatoSPI,0);
                                }
                                 //Son 455 datos en total
                                //Haciendo = SPI_SlaveTransfer(idx,0); //Son 455 datos en total
                            }
                            Haciendo=0;
                            ss=0;
                            break;

                default:    Haciendo=0; break;
            }
        }



  //Esperar comando del serial
/*  while((UCSRA&0x80)!=0x80){}; //bandera RXC
  caracter=UDR;
  if(caracter=='$') //Si comando de inicio...
  {
   UDR='$'; //comenzar transmision de la cadena

   MideF(0,0,4); //rojo Texp=2ms
   while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
   EnviaHex(conteo);//RGBW[0]; //enviar valor Rojo
   MideF(0,1,4); //verde Texp=2ms
   while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
   EnviaHex(conteo);//RGBW[0]; //enviar valor verde
   MideF(0,2,4); //azul Texp=2ms
   while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
   EnviaHex(conteo);//RGBW[0]; //enviar valor azul
   MideF(0,3,2); //blanco Texp=1ms
   while(banConteo!=1){}; //esperar mientras se hace la medición del sensor 0
   EnviaHex(conteo);//RGBW[0]; //enviar valor blanco

   delayus(450); //esperar 500us
   UDR='$'; //finalizar transmision de la cadena
   while((UCSRA&0x40)!=0x40){}; //bandera TXC 1.044ms aprox
   UCSRA|=0x40; //apagar bandera TXC
    */


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
  //}
 }


}



//!********************************************************** FUNCIONES ***************************************************


//!---------------------------------------------- funcion que mide la frecuencia del pin CH ------------------------------
void MideF(int color, int Texp)
{
 //sleccionar color
 //S3=PA1 S2=PA2   Color
 //  0      0      Rojo
 //  0      1      Blanco
 //  1      0      Azul
 //  1      1      Verde
 switch(color)
 {
  case 0: PORTA &= ~0x02; //PA1=0 S3=0 rojo
  	   	  PORTA &= ~0x04; //PA2=0 S2=0
		  break;
  case 1: PORTA |=  0x02; //PA1=1 S3=1 verde
  	   	  PORTA |=  0x04; //PA2=1 S2=1
		  break;
  case 2: PORTA |=  0x02; //PA1=1 S3=1 azul
  	   	  PORTA &= ~0x04; //PA2=0 S2=0
	      break;
  case 3: PORTA &= ~0x02; //PA1=0 S3=0 blanco
  	   	  PORTA |=  0x04; //PA2=1 S2=1
		  break;
  default: break;
 }

 //inicia la medicion
 banConteo=0;
 conteo=0;
 //iniciar valor del conteo timer 1 en 0s

 /*TCCR1B = 0x00; //stop timer1
 TCCR0B = 0x00; //stop timer0*/


//---tiempo de exposicion ------------------------
 //Texp=1 500us, Texp=2 1ms, Texp=3 1.5ms, Texp=4 2ms
//TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
 //Seleccionar sensor
 switch(Texp)
 {
  case 1:   //ventana de tiempo 500us a 20Mhz
  	   		OCR1AH = 0x00;  //
  	   		OCR1AL = 0x27;
  	   		TCNT1H=0x00;//Reset Timer
            TCNT1L=0x00;//Reset Timer
 			//OCR1B = 0x001F;  //
 			//TCNT0 = 0xE1;  //
			break;
  case 2:   //ventana de tiempo 1ms a 20Mhz
  	   		OCR1AH = 0x00;  //
  	   		OCR1AL = 0x4E;
  	   		TCNT1H=0x00;//Reset Timer
            TCNT1L=0x00;//Reset Timer
 			//OCR1B = 0x003E;  //
 			//TCNT0 = 0xC2;  //
			break;
  case 3:   //ventana de tiempo para luz solar a 20Mhz
  	   		OCR1A = 0x16;
  	   		TCNT1H=0x00;//Reset Timer
            TCNT1L=0x00;//Reset Timer
 			//OCR1B = 0x0076;  //
 			//TCNT0 = 0xA2;  //
			break;
  case 4:   //ventana de tiempo 22ms a 20Mhz
  	   		OCR1A = 0x6B7;
  	   		TCNT1H=0x00;//Reset Timer
            TCNT1L=0x00;//Reset Timer
 			//OCR1B = 0x009C;  //
 			//TCNT0 = 0x83;  //
			break;
  default: break;
 }


TCNT0=0x00;//Reset Counter

TCCR1B |= (1 << CS12); //start timer CLK/256
//TCCR1B |= (1 << WGM12);
//comenzar conteo Timer0
//TCCR0B = 0x07; //start Timer T0 entrada de pulsos
TCCR0B |= ((1 << CS12) | (1 << CS11) | (1 << CS10));
SEI(); //re-enable interrupts
}

//!--------------------------------------------- Funcion para corregir colores raw -----------------------------------------
void CorrigeSensorColor(int Rr, int Gr, int Br)
{
    int Ra,Ga,Ba;
    float Hr,Sr,Vr;
    int maxRGB, minRGB, deltaRGB;
    //int I,F,P,Q,T;

    // Balance de Negros (offset)
    if(BalanceNegros==1)
    {
        Rr=(Rr*rslm)+(rslb);
        Gr=(Gr*gslm)+(gslb);
        Br=(Br*bslm)+(bslb);
    }
    // Verificar amplitudes
    if(Rr<0)   Rr=0;
    if(Rr>255) Rr=255;
    if(Gr<0)   Gr=0;
    if(Gr>255) Gr=255;
    if(Br<0)   Br=0;
    if(Br>255) Br=255;

    // Correccion Gama (exponencial)
    /*if(CorregirGama==1)
    {
        Rr=Gama[Rr&0x00FF];
        Gr=Gama[Gr&0x00FF];
        Br=Gama[Br&0x00FF];
    }*/

    // Correccion de Color
    //MC es la matriz de correccion de color
    if(CorregirColor==1)
    {
        Ra=Rr; Ga=Gr; Ba=Br;
        Rr=(Ra*mcrm)+(mcrb);
        Gr=(Ga*mcgm)+(mcgb);
        Br=(Ba*mcbm)+(mcbb);
        /*
        Rr = (MC11*Ra + MC12*Ga + MC13*Ba )/255;
        Gr = (MC21*Ra + MC22*Ga + MC23*Ba )/255;
        Br = (MC31*Ra + MC32*Ga + MC33*Ba )/255;*/
    }

    // Verificar amplitudes
    if(Rr<0)   Rr=0;
    if(Rr>255) Rr=255;
    if(Gr<0)   Gr=0;
    if(Gr>255) Gr=255;
    if(Br<0)   Br=0;
    if(Br>255) Br=255;

    //Conversion HSV
    //maxRGB=max(R,G,B);
    if(Rr>=Gr) maxRGB=Rr;
    else maxRGB=Gr;
    if(maxRGB<=Br) maxRGB=Br;

    //minRGB=min([R,G,B]);
    if(Rr<=Gr) minRGB=Rr;
    else minRGB=Gr;
    if(minRGB>=Br) minRGB=Br;

    deltaRGB = maxRGB - minRGB;
    Vr=maxRGB;

    if(maxRGB==0)
    {
        Sr=0;
        Hr=0;
    }
    else
    {
        Sr=(deltaRGB*255)/maxRGB;
        if(maxRGB==Rr)
        {
            if(deltaRGB!=0) Hr=((Gr-Br)*60)/deltaRGB +0;
            else Hr=0;
        }
        else
        {
            if(maxRGB==Gr)
            {
                if(deltaRGB!=0) Hr=((Br-Rr)*60)/deltaRGB + 120;
                else Hr=0;
            }
            else
            {
                if(deltaRGB!=0) Hr=((Rr-Gr)*60)/deltaRGB + 240;
                else Hr=0;
            }
        }
    }
    if(Hr<0) Hr=Hr+360;

    if(Hr==0 && Sr==0 && Vr==0){
        CorregirHSV=0;
    }
    //Correcion de H_SV
    if(CorregirHSV==1)
    {
        Ha=Hr/360; Sa=Sr/255; Va=Vr/255;
        Hr = (Ha*mchm)+(mchb);
        Sr = (Sa*mcsm)+(mcsb);
        Vr = (Va*mcvm)+(mcvb);
        if(Hr<0)     Hr=0;
        if(Hr>1)   Hr=1;
        if(Sr<0)     Sr=0;
        if(Sr>1)   Sr=1;
        if(Vr<0)     Vr=0;
        if(Vr>1)   Vr=1;

        Hr=Hr*360;
        Sr=Sr*255;
        Vr=Vr*255;
    }

    // Verificar amplitudes
    if(Hr<0)     Hr=0;
    if(Hr>360)   Hr=360;
    if(Sr<0)     Sr=0;
    if(Sr>255)   Sr=255;
    if(Vr<0)     Vr=0;
    if(Vr>255)   Vr=255;


    //%SVP=SPa.*VPa;
    //%SVC=SCa.*VCa;

    // Obtener RGB a partir de HSV corregido
    /*if(ObtenerRGBdeHSV==1)
    {
        if(Sr>umbralGris) //umbral de croma/BW
        {
            I=floor(Hr/60); //Parte entera, sector 0 - 5
            Ha=(Hr*1000/60);
            F=Ha-I*1000;

            P=Vr*(255-Sr)/255;
            Q=Vr*(255*1000-Sr*F)/(255*1000);
            T=Vr*(255*1000-Sr*(1000-F))/(255*1000);
            switch(I)
            {
                case 0:
                    Rr=Vr; Gr=T; Br=P;
                    break;
                case 1:
                    Rr=Q; Gr=Vr; Br=P;
                    break;
                case 2:
                    Rr=P; Gr=Vr; Br=T;
                    break;
                case 3:
                    Rr=P; Gr=Q; Br=Vr;
                    break;
                case 4:
                    Rr=T; Gr=P; Br=Vr;
                    break;
                case 5:
                    Rr=Vr; Gr=P; Br=Q;
                    break;
                default:
                    break;
            }
        }
        else
            Rr=Vr; Gr=Vr; Br=Vr;           //Escala de grises
    }

    // Verificar amplitudes
    if(Rr<0)   Rr=0;
    if(Rr>255) Rr=255;
    if(Gr<0)   Gr=0;
    if(Gr>255) Gr=255;
    if(Br<0)   Br=0;
    if(Br>255) Br=255;*/

    R=Rr; G=Gr; B=Br;
    H=Hr; S=Sr; V=Vr;
}



//!------------------------------------- Cargar todos los parametros de correccion desde la EEPROM -------------------------
void CargaParametros(void)
{
    //Comandos de correccion de color
    BalanceNegros=EEPROM_read(1);      //EEPROM[1]
    BalanceBlancos=EEPROM_read(2);     //EEPROM[2]
    CorregirGama=EEPROM_read(3);       //EEPROM[3]
    CorregirColor=EEPROM_read(4);      //EEPROM[4]
    CorregirHSV=EEPROM_read(5);        //EEPROM[5]
    ObtenerRGBdeHSV=EEPROM_read(6);    //EEPROM[6]

    //Parametros de correccion de color
    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(9+i);
    }
	memcpy(&rslb,&storage,4);
    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(13+i);
    }
	memcpy(&rslm,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(17+i);
    }
	memcpy(&gslb,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(21+i);
    }
	memcpy(&gslm,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(25+i);
    }
	memcpy(&bslb,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(29+i);
    }
	memcpy(&bslm,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(33+i);
    }
	memcpy(&mcrb,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(37+i);
    }
	memcpy(&mcrm,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(41+i);
    }
	memcpy(&mcgb,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(45+i);
    }
	memcpy(&mcgm,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(49+i);
    }
	memcpy(&mcbb,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(53+i);
    }
	memcpy(&mcbm,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(57+i);
    }
	memcpy(&mchb,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(61+i);
    }
	memcpy(&mchm,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(65+i);
    }
	memcpy(&mcsb,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(69+i);
    }
	memcpy(&mcsm,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(73+i);
    }
	memcpy(&mcvb,&storage,4);

    for(int i=0;i<4;i++){
        storage[i]=EEPROM_read(77+i);
    }
	memcpy(&mcvm,&storage,4);
    //Matriz de correccion HSV
    MHSV11=EEPROM_read(200)<<8;           //Parte alta EEPROM[38:39]
    MHSV11 |= EEPROM_read(201);           //Parte baja
    MHSV12=EEPROM_read(202)<<8;           //Parte alta EEPROM[40:41]
    MHSV12 |= EEPROM_read(203);           //Parte baja
    MHSV13=EEPROM_read(204)<<8;           //Parte alta EEPROM[42:43]
    MHSV13 |= EEPROM_read(205);           //Parte baja
    MHSV21=EEPROM_read(206)<<8;           //Parte alta EEPROM[44:45]
    MHSV21 |= EEPROM_read(207);           //Parte baja
    MHSV22=EEPROM_read(208)<<8;           //Parte alta EEPROM[46:47]
    MHSV22 |= EEPROM_read(209);           //Parte baja
    MHSV23=EEPROM_read(210)<<8;           //Parte alta EEPROM[48:49]
    MHSV23 |= EEPROM_read(211);           //Parte baja
    MHSV31=EEPROM_read(212)<<8;           //Parte alta EEPROM[50:51]
    MHSV31 |= EEPROM_read(213);           //Parte baja
    MHSV32=EEPROM_read(214)<<8;           //Parte alta EEPROM[52:53]
    MHSV32 |= EEPROM_read(215);           //Parte baja
    MHSV33=EEPROM_read(216)<<8;           //Parte alta EEPROM[54:55]
    MHSV33 |= EEPROM_read(217);           //Parte baja






    /*for(i=200;i<456;i++)
    {
        Gama[i-200]=EEPROM_read(i);          //EEPROM[200:455]
    }*/
}

//!------------------------------------------------ Transferencia SPI -----------------------------------------------------
uint8_t SPI_SlaveTransfer(uint8_t Dato, char llamada)
{
    //dato a Transferir
    USIDR=Dato;
    if(llamada==1)
    {
        //falta esperar a que el pin no sea cero debido a otro dispositivo en la red (checar, no creo que funcione)
        //Indicar Ready
        DDRB |= 0x04;  //poner como salida PB2 (INT0)
        PORTB&=~0x04;  //PB2=0 (INT0)
    }
    //Limpiar bandera counter overflow y contador para comenzar envio
	USISR = (1<<USIOIF);
    //Esperar a que se termine la transaccion
    while((USISR&(1<<USIOIF))!=(1<<USIOIF)){};
    if(llamada==1)
    {
        //Indicar busy
        DDRB &= ~0x04; //poner como entrada PB2 (INT0)
        PORTB|= 0x04;  //Pull up PB2 (INT0)
    }
    //Leer dato recibido
    Dato=USIDR;
    return Dato;
}

//!--------------------------------------------- Leer un byte de EEprom ---------------------------------------------------
uint8_t EEPROM_read(uint16_t ucAddress)
{
    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE)){};
    /* Set up address register */
    EEAR = ucAddress;
    /* Start eeprom read by writing EERE */
    EECR |= (1<<EERE);
    /* Return data from data register */
    return EEDR;
}

//!------------------------------------------- Escribir un byte en EEprom -------------------------------------------------
void EEPROM_write(uint16_t ucAddress, uint8_t ucData)
{
    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE)){};
    /* Set Programming mode */
    EECR = (0<<EEPM1)|(0<<EEPM0);
    /* Set up address and data registers */
    EEAR = ucAddress;
    EEDR = ucData;
    /* Write logical one to EEMPE */
    EECR |= (1<<EEMPE);
    /* Start eeprom write by setting EEPE */
    EECR |= (1<<EEPE);
}

//!--------------------------------------------- Configuracion de perifericos -----------------------------------------------
void Config(void)
{
    //stop errant interrupts until set up
    CLI(); //disable all interrupts

    //Configurar SPI SLAVE en SPI, reloj externo ambos flancos, sin interrupcion
    USICR=(1<<USIWM0)|(1<<USICS1);
    //Poner DO(PA5) en DDRA como salida
    DDRA|=0x20;
    //Pull ups en MOSI(DI)(PA6) y USCK(PA4)
    PORTA|=(0x40)|(0x01);

    //configurar puertos
    DDRA|= 0x06; //PA2 y PA1 para seleccion de color
    PORTB|=0x08 | 0x04; //Pull up PB3 (RESET), PB2 (INT0)
    //Configurar int0
    MCUCR |= 0x02;
    GIMSK |= 0x40;
    //Configurar timers
    TIMSK1 |= 0x02;
    TIMSK0 |= (1<<TOIE0);

    //TCCR1A = 0x02;
    TCCR1B |= (1 << WGM12);



    //CargaParametros();

    /*
  //configurar timer 0
 //TIMER0 initialize - prescale:64
// WGM: Normal
// desired value: 2KHz
// actual value:  2,000KHz (0,0%)
 TCCR0B = 0x00; //stop timer
 TCCR0A = 0x00; //start timer ok, no mover
 OCR0A = 0x7D;  //Valor para ventana de tiempo de 500us
 OCR0B = 0x7D;  //""
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

 // //re-enable interrupts
 //all peripherals are now initialized
 */
 SEI();
}

void delayus(unsigned int usec)
{
 unsigned int i;
 usec=usec*3;
 for(i=0; i<(usec);i++){};
}

void delay_ms(uint32_t msec)
{
    uint32_t j;
	for(j=0;j<(msec*100);j++);
}

/*
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
 //UDR=VHL[1]; //Parte alta
 //while((UCSRA&0x40)!=0x40){}; //bandera TXC 1.044ms aprox
 //UCSRA|=0x40; //apagar bandera TXC
 //delayus(450); //esperar 500us
 //UDR=VHL[0]; //Parte baja
 //while((UCSRA&0x40)!=0x40){}; //bandera TXC 1.044ms aprox
 //UCSRA|=0x40; //apagar bandera TXC
 //aqui no conviene esperar, tanto
 //el tiempo de adquisicion del siguiente dato es de 1/2ms
}
*/














//-------------------------------------borrar


void GuardaParametros(void)
{
    //Comandos de correccion de color
    EEPROM_write(1, BalanceNegros);
    EEPROM_write(2, BalanceBlancos);
    EEPROM_write(3, CorregirGama);
    EEPROM_write(4, CorregirColor);
    EEPROM_write(5, CorregirHSV);
    EEPROM_write(6, ObtenerRGBdeHSV);

	 //Parametros de correccion de color
	memcpy(storage,&rslb,4);
	for(int i=0;i<4;i++){
		 EEPROM_write(9+i,storage[i]);
	}
	memcpy(storage,&rslm,4);
	for(int i=0;i<4;i++){
		EEPROM_write(13+i,storage[i]);
	}
	memcpy(storage,&gslb,4);
	for(int i=0;i<4;i++){
		EEPROM_write(17+i,storage[i]);
	}
	memcpy(storage,&gslm,4);
	for(int i=0;i<4;i++){
		EEPROM_write(21+i,storage[i]);
	}
	memcpy(storage,&bslb,4);
	for(int i=0;i<4;i++){
		EEPROM_write(25+i,storage[i]);
	}
	memcpy(storage,&bslm,4);
	for(int i=0;i<4;i++){
		EEPROM_write(29+i,storage[i]);
	}
	memcpy(storage,&mcrb,4);
	for(int i=0;i<4;i++){
		EEPROM_write(33+i,storage[i]);
	}
	memcpy(storage,&mcrm,4);
	for(int i=0;i<4;i++){
		EEPROM_write(37+i,storage[i]);
	}
	memcpy(storage,&mcgb,4);
	for(int i=0;i<4;i++){
		EEPROM_write(41+i,storage[i]);
	}
	memcpy(storage,&mcgm,4);
	for(int i=0;i<4;i++){
		EEPROM_write(45+i,storage[i]);
	}
	memcpy(storage,&mcbb,4);
	for(int i=0;i<4;i++){
		EEPROM_write(49+i,storage[i]);
	}
	memcpy(storage,&mcbm,4);
	for(int i=0;i<4;i++){
		EEPROM_write(53+i,storage[i]);
	}
	memcpy(storage,&mchb,4);
	for(int i=0;i<4;i++){
		EEPROM_write(57+i,storage[i]);
	}
	memcpy(storage,&mchm,4);
	for(int i=0;i<4;i++){
		EEPROM_write(61+i,storage[i]);
	}
	memcpy(storage,&mcsb,4);
	for(int i=0;i<4;i++){
		EEPROM_write(65+i,storage[i]);
	}
	memcpy(storage,&mcsm,4);
	for(int i=0;i<4;i++){
		EEPROM_write(69+i,storage[i]);
	}
	memcpy(storage,&mcvb,4);
	for(int i=0;i<4;i++){
		EEPROM_write(73+i,storage[i]);
	}
	memcpy(storage,&mcvm,4);
	for(int i=0;i<4;i++){
		EEPROM_write(77+i,storage[i]);
	}

    //Matriz de correccion HSV
    EEPROM_write(200, MHSV11>>8);          //parte alta
    EEPROM_write(201, (MHSV11&0x00FF));    //parte baja
    EEPROM_write(202, MHSV12>>8);          //parte alta
    EEPROM_write(203, (MHSV12&0x00FF));    //parte baja
    EEPROM_write(204, MHSV13>>8);          //parte alta
    EEPROM_write(205, (MHSV13&0x00FF));    //parte baja
    EEPROM_write(206, MHSV21>>8);          //parte alta
    EEPROM_write(207, (MHSV21&0x00FF));    //parte baja
    EEPROM_write(208, MHSV22>>8);          //parte alta
    EEPROM_write(209, (MHSV22&0x00FF));    //parte baja
    EEPROM_write(210, MHSV23>>8);          //parte alta
    EEPROM_write(211, (MHSV23&0x00FF));    //parte baja
    EEPROM_write(212, MHSV31>>8);          //parte alta
    EEPROM_write(213, (MHSV31&0x00FF));    //parte baja
    EEPROM_write(214, MHSV32>>8);          //parte alta
    EEPROM_write(215, (MHSV32&0x00FF));    //parte baja
    EEPROM_write(216, MHSV33>>8);          //parte alta
    EEPROM_write(217, (MHSV33&0x00FF));    //parte baja

}



//****************************** INTERRUPCIONES ********************************
//-------------------------- FUNCIONES DE INTERRUPCION -------------------------
//interrupcion externa 0
#pragma interrupt_handler int0_isr:2
void int0_isr(void){
    ss=1;
}
//Pin change interrupt 0
#pragma interrupt_handler PinCh0_int_isr:3
void PinCh0_int_isr(void)
{ }
//Pin change interrupt 1
#pragma interrupt_handler PinCh1_int_isr:4
void PinCh1_int_isr(void)
{ }
//wdt overflow
#pragma interrupt_handler wdt_ovf_isr:5
void wdt_ovf_isr(void)
{ }
//timer 1 input capture
#pragma interrupt_handler timer1_capt_isr:6
void timer1_capt_isr(void)
{ }
//timer 1 comparacion igual A
#pragma interrupt_handler timer1_compa_isr:7
void timer0_compa_isr(void)
{ }
//timer 1 comparacion igual B
#pragma interrupt_handler timer1_compb_isr:8
void timer1_compb_isr(void)
{ }
#pragma interrupt_handler timer1_ovf_isr: 9
void timer1_ovf_isr(void)
{ }
#pragma interrupt_handler timer0_compa_isr:10 //iv_TIM0_COM
void timer1_compa_isr(void)
{
 //termino el tiempo de comparacion
 //TCCR1B = 0x00; //stop timer1
 TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));

 //TCCR0B = 0x00; //stop timer0
 TCCR0B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));

 //TCCR1A = 0x00;

 if(banConteo==0) //no hay overflow
 {
  conteo=TCNT0;
 }
 else //overflow: frecia mayor a la capacidad de timer1
 {
  conteo=0xFF;
 }
 banConteo=1;
 CLI();
}
//timer 0 comaracion igual B
#pragma interrupt_handler timer0_compb_isr:11
void timer0_compb_isr(void)
{ }
//timer 0 overflow
#pragma interrupt_handler timer0_ovf_isr:12
void timer0_ovf_isr(void)
{
 //TCCR0B = 0x00; //stop timer0
 TCCR0B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); //stop timer
 //conteo=0xFF;
 ovf++;
 TCNT0=0;
 banConteo=2;   //conteo de frecuencia sobrepasa la capacidad del timer 0
}
//comparador analogico
#pragma interrupt_handler ana_comp_isr:13
void ana_comp_isr(void)
{ }
//conversion lista ADC
#pragma interrupt_handler adc_isr:14
void adc_isr(void)
{ }
//EEPROM listo
#pragma interrupt_handler eeprom_ready_isr:15
void eeprom_ready_isr(void)
{ }
//USI start
#pragma interrupt_handler usi_start_isr:16
void usi_start_isr(void)
{ }
//USI overflow
#pragma interrupt_handler usi_ovf_isr:17
void usi_ovf_isr(void)
{ }









