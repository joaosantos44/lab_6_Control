//******************************************************************************
/*
Universidad del Valle de Guatemala
IE3054: Electronica Digital 2
Lab_1
Autor; Joao Santos
Lab_1
Hardware: Atemega328P
Creado: 28/01/2025
Ultima modificacion: 02/02/2025
*/
//****************************************************************************** 

//******************************************************************************
//LIBRERIAS
//******************************************************************************
#define F_CPU 16000000

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

//******************************************************************************
//Variables
//******************************************************************************

volatile char indic;

//Variable del boton de Arriba
uint8_t ari = 0;
//Variable del boton de Abajo
uint8_t aba = 0;
//Variable del boton de A
uint8_t boA = 0;
//Variable del boton de B
uint8_t boB = 0;
//Variable del boton de Derecha
uint8_t der = 0;
//Variable del boton de Izquierda
uint8_t izq = 0;


//******************************************************************************
//Prototipos de Funcion
//******************************************************************************
void Setup(void);
void initUART9600(void);
void initUART115200(void);
void floatToString(float num, char *str, int precision);
void MandarP(char letra);
void PINCHANGE(void);

//******************************************************************************
//Codigo
//******************************************************************************

int main(void)
{
	Setup();
	
	while (1)
	{	
		if(ari){
			CadenChar("U");
			ari = 0;
		}
		
		if(aba){
			CadenChar("D");
			aba = 0;
		}
		
		if(der){
			CadenChar("E");
			der = 0;
		}
		
		if(izq){
			CadenChar("I");
			izq = 0;
		}
		
		if(boA){
			CadenChar("A");
			boA = 0;
		}
		
		if(boB){
			CadenChar("B");
			boB = 0;
		}
	
	}
	
}



//******************************************************************************
//Setup
//******************************************************************************
 
void Setup(void){
	//setear los pines C0 a C5 Como entrada
	DDRC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3) | (1 << PINC4) | (1 << PINC5));
	//Pullup
	PORTC |= (1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3) | (1 << PINC4) | (1 << PINC5);	

	initUART9600();
	PINCHANGE();
	
	sei();
}


//******************************************************************************
//Botones
//******************************************************************************

void PINCHANGE(void){
	// Activar las interupciones de PC1 (Puerto C)
	PCICR |= (1 << PCIE1);
	// Activar las interupciones de PC1 (Pines 0C, 1C, 2C, 3c, 4c y 5c)
	PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13);
}

//******************************************************************************
//Uart
//******************************************************************************

void initUART9600(void){
	uint16_t ubrr = F_CPU / 16 / 9600 - 1;
	UBRR0H = (ubrr >> 8);   // Parte alta del baud rate
	UBRR0L = ubrr;          // Parte baja del baud rate
	UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Habilita TX y RX
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 bits de datos
}

void initUART115200(void){
	uint16_t ubrr = F_CPU / 16 / 115200 - 1; // Calcula el baudrate para 115200
	UBRR0H = (ubrr >> 8);   // Parte alta del baud rate
	UBRR0L = ubrr;          // Parte baja del baud rate
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); // Habilita TX, RX e interrupción RX
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 bits de datos
}

void floatToString(float num, char *str, int preci) {
	
	// Parte entera
	int ENPart = (int)num;
	// Parte decimal
	float deciPart = num - ENPart;

	// Convertir parte entera con itoa()
	int centena = ENPart / 100;
	int decena = (ENPart / 10) % 10;
	int unidad = ENPart % 10;

	// Agregar punto decimal
	int i = 0;
	while (str[i] != '\0') i++;  // Buscar Nulo
	str[i++] = '.';

	// Convertir parte decimal manualmente
	for (int j = 0; j < preci; j++) {
		deciPart *= 10;
		int decDigit = (int)deciPart;
		str[i++] = decDigit + '0';
		deciPart -= decDigit;
	}
	
	// Terminar la cadena
	str[i] = '\0';
}

/*
void floatToString(float num, char *str, int preci) {
	int ENPart = (int)num;
	float deciPart = num - ENPart;

	int centena = ENPart / 100;
	decena = (ENPart / 10) % 10;
	unidad = ENPart % 10;

	// Convertimos a caracteres


	int i = 0;
	if (centena > 0) {
		str[i++] = centena + '0';
	}
	str[i++] = '.';

	for (int j = 0; j < preci; j++) {
		deciPart *= 10;
		int decDigit = (int)deciPart;
		str[i++] = decDigit + '0';
		deciPart -= decDigit;
	}

	str[i] = '\0';
}
*/


// Envía un caracter
void MandarP(char letra){
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = letra;
}


//recive un caracter
char ReceivP(void) {
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}


// Envía una cadena de texto
void CadenChar(const char *str) {
	while (*str) {
		MandarP(*str++);
	}
}

// Convierte un número a binario
void Binario(uint8_t valor) {
	for (int i = 7; i >= 0; i--) {
		MandarP((valor & (1 << i)) ? '1' : '0');
	}
	CadenChar("\r\n");
}

void UART_SendHex(uint8_t value) {
	char hexStr[5];
	sprintf(hexStr, "%02X", value);  // Convierte a hexadecimal
	CadenChar(hexStr);
}

int InterDeTeclas() {
	char buffer[4] = {0};   // Almacena hasta 3 dígitos
	uint8_t index = 0;
	char received;

	while (1) {
		while (!(UCSR0A & (1 << RXC0)));  // Espera un dato (¡Aquí puede trabarse!)
		
		received = UDR0;  // Lee el dato recibido
		MandarP(received); // Imprime lo que recibe (eco)

		// Si encuentra '+', termina
		if (received == '+') {
			break;
		}

		// Acepta solo dígitos (0-9)
		if (received >= '0' && received <= '9' && index < 3) {
			buffer[index++] = received;  // Guarda el número
		}
	}

	return (uint8_t)atoi(buffer);  // Convierte a entero
}



//********************************************************************************************************************************************
// Interrupciones
//********************************************************************************************************************************************


ISR(USART_RX_vect){
	//indic = UDR0; // Leer el dato recibido
		
}

ISR(PCINT1_vect){
	//Valores para los botones
	uint8_t botAR = PINC & (1 << PINC0);
	uint8_t botAB = PINC & (1 << PINC3);
	uint8_t botDE = PINC & (1 << PINC1);
	uint8_t botIZ = PINC & (1 << PINC4);
	uint8_t botA = PINC & (1 << PINC2);
	uint8_t botB = PINC & (1 << PINC5);


	//Contador para el boton PC0
	if (botAR == 0){
		ari = 1;
	}
	
	//Contador para el boton PC3
	if (botAB == 0){
		aba = 1;
	}
	
	//Contador para el boton PC1
	if (botDE == 0){
		der = 1;
	}
	
	//Contador para el boton PC4
	if (botIZ == 0){
		izq = 1;
	}
	
	//Contador para el boton PC2
	if (botA == 0){
		boA = 1;
	}
	
	//Contador para el boton PC5
	if (botB == 0){
		boB = 1;
	}
	
	// Desactivado de banderas
	PCIFR |= (1 << PCIF1);
}
