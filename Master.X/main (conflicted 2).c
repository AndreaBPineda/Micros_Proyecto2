/*
 * File:            main.c
 * Author:          Andrea Barrientos Pineda, 20575
 *
 * Creado:          23/05/2022
 * Modificado:      02/06/2022
 * 
 * Proyecto 2       Programa PIC Maestro - Lectura y envío de datos
 */

//-------------------- Config 1 ------------------------------------------------
//------------------------------------------------------------------------------
#pragma config FOSC = INTRC_NOCLKOUT
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config MCLRE = OFF
#pragma config CP = OFF
#pragma config CPD = OFF
#pragma config BOREN = OFF
#pragma config IESO = OFF
#pragma config FCMEN = OFF
#pragma config LVP = OFF

//-------------------- Config 2 ------------------------------------------------
//------------------------------------------------------------------------------
#pragma config BOR4V = BOR40V
#pragma config WRT = OFF

//-------------------- Librerías -----------------------------------------------
//------------------------------------------------------------------------------
#include <xc.h>
#include <stdint.h>
#include <stdio.h>

//-------------------- Constantes ----------------------------------------------
//------------------------------------------------------------------------------
#define _XTAL_FREQ  4000000             // Constante para __delay_us

//-------------------- Variables -----------------------------------------------
//------------------------------------------------------------------------------
uint8_t data_SPI = 0;
uint8_t position_servo = 0;

uint8_t flag_EEPROM = 0;
uint8_t flag_EUSART = 0;

uint8_t servo1 = 0;
uint8_t servo2 = 0;
uint8_t servo3 = 0;
uint8_t servo4 = 0;

uint8_t servo3_READ = 0;
uint8_t servo4_READ = 0;
uint8_t read_serial = 0;

//-------------------- Declaración de funciones --------------------------------
//------------------------------------------------------------------------------
void setup(void);                                   // Configuración del PIC
void reset_TMR0(void);                              // Resetear TMR0
void write_EEPROM(uint8_t address, uint8_t data);   // Escritura EEPROM
void print(char str[]);                             // Imprimir en terminal
void run_EUSART(void);                              // Correr EUSART

uint8_t read_EEPROM(uint8_t address);               // Lectura EEPROM

//-------------------- Interrupciones ------------------------------------------
//------------------------------------------------------------------------------
void __interrupt() isr (void)           
{
    // Interrupcion ADC
    if (PIR1bits.ADIF)                  
    {
        // Servo 1
        if (ADCON0bits.CHS == 0b0000)
        {
            servo1 = ADRESH;
            data_SPI = ADRESH;          // Guardar valor de la conversion
            PORTDbits.RD0 = 1;          // Selector Servo 1: encendido
            PORTDbits.RD1 = 0;          // Selector Servo 2: apagado
        }
        
        // Servo 2
        else if (ADCON0bits.CHS == 0b0001)
        {
            servo2 = ADRESH;
            data_SPI = ADRESH;          // Guardar valor de la conversion
            PORTDbits.RD0 = 0;          // Selector Servo 1: apagado
            PORTDbits.RD1 = 1;          // Selector Servo 2: encendido
        }
        
        // Servo 3
        else if (ADCON0bits.CHS == 0b0010)
        {
            servo3 = ADRESH;
            CCPR1L = (ADRESH>>1)+123;
            CCP1CONbits.DC1B = (ADRESH & 0b01);
            CCP1CONbits.DC1B0 = (ADRESH>>7);
        }
        
        // Servo 4
        else if (ADCON0bits.CHS == 0b0011)
        {
            servo4 = ADRESH;
            CCPR2L = (ADRESH>>1)+123;
            CCP1CONbits.DC1B = (ADRESH & 0b01);
            CCP1CONbits.DC1B0 = (ADRESH>>7);
        }
        
        PIR1bits.ADIF = 0;              // Desactivar interrupcion
    }
    
    // Interrupcion TMR0
    if (INTCONbits.T0IF)                
    {
        reset_TMR0();                   // Resetear TMR0
        INTCONbits.T0IF = 0;            // Desactivar interrupcion
    }
    
    if (INTCONbits.RBIF)
    {
        // Pushbutton 1: Modo de funcionamiento Escritura/Lectura
        if (!PORTBbits.RB0)
        {
            flag_EEPROM = !flag_EEPROM;  // Invertir valor de la bandera
            PORTAbits.RA6 = flag_EEPROM;
            ADCON0bits.ADON = flag_EEPROM;
            __delay_ms(10);
        }
        
        // Pushbutton 2: Posicion 1 a registrar
        else if (!PORTBbits.RB1)
        {
            if (flag_EEPROM)             // 1: Registrar posicion 1
            {   
                write_EEPROM(0x00, servo1);
                write_EEPROM(0x01, servo2);
                write_EEPROM(0x02, servo3);
                write_EEPROM(0x03, servo4);
            }
            
            else
            {
                data_SPI = read_EEPROM(0x00);
                PORTDbits.RD0 = 1;
                PORTDbits.RD0 = 0;
                SSPBUF = data_SPI;
                
                __delay_ms(10);
                
                data_SPI = read_EEPROM(0x01);
                PORTDbits.RD0 = 0;
                PORTDbits.RD0 = 1;
                SSPBUF = data_SPI;
                
                __delay_ms(10);
                
                servo3_READ = read_EEPROM(0x02);
                CCPR1L = (servo3_READ>>1)+123;
                CCP1CONbits.DC1B = (servo3_READ & 0b01);
                CCP1CONbits.DC1B0 = (servo3_READ>>7);
                
                servo4_READ = read_EEPROM(0x03);
                CCPR2L = (servo4_READ>>1)+123;
                CCP1CONbits.DC1B = (servo4_READ & 0b01);
                CCP1CONbits.DC1B0 = (servo4_READ>>7);
            }
        }
        
        // Pushbutton 3: Posicion 2 a registrar
        else if (!PORTBbits.RB2)
        {
            if (flag_EEPROM)            // 1: Registrar posicion 2
            {
                write_EEPROM(0x04, servo1);
                write_EEPROM(0x05, servo2);
                write_EEPROM(0x06, servo3);
                write_EEPROM(0x07, servo4);
            }
            
            else
            { 
                data_SPI = read_EEPROM(0x04);
                PORTDbits.RD0 = 1;
                PORTDbits.RD0 = 0;
                SSPBUF = data_SPI;
                
                __delay_ms(10);
                
                data_SPI = read_EEPROM(0x05);
                PORTDbits.RD0 = 0;
                PORTDbits.RD0 = 1;
                SSPBUF = data_SPI;
                
                __delay_ms(10);
                
                servo3_READ = read_EEPROM(0x06);
                CCPR1L = (servo3_READ>>1)+123;
                CCP1CONbits.DC1B = (servo3_READ & 0b01);
                CCP1CONbits.DC1B0 = (servo3_READ>>7);
                
                servo4_READ = read_EEPROM(0x07);
                CCPR2L = (servo4_READ>>1)+123;
                CCP1CONbits.DC1B = (servo4_READ & 0b01);
                CCP1CONbits.DC1B0 = (servo4_READ>>7);
            }
        }
        
        else if (!PORTBbits.RB3)
        {
            flag_EUSART = !flag_EUSART;     // Invertir valor de la bandera
            PORTAbits.RA5 = flag_EUSART;    // Led indicador EUSART
        }
        
        INTCONbits.RBIF = 0;            // Desactivar interrupcion
    }
    
    // Interrupcion SPI
    if (PIR1bits.SSPIF)
    {
        PIR1bits.SSPIF = 0;             // Desactivar interrupcion
    }
    
    if (PIR1bits.RCIF)                  // Recepción de datos
    { 
        read_serial = RCREG;            // Almacenar valor ingresado
    }
}

//-------------------- Programa principal --------------------------------------
//------------------------------------------------------------------------------

void main(void)                         
{
    setup();                            
    reset_TMR0();                       
    __delay_ms(10);
    
    while (1)                         
    {
        if (flag_EUSART)
        {
            run_EUSART();
        }
        
        // Control de canales e inicio de conversion ADC
        if (ADCON0bits.GO == 0)         
        {
            if (ADCON0bits.CHS == 0b0000)           // Canal 0000 -> 0001
            {
                ADCON0bits.CHS = 0b0001;
            }
            
            else if (ADCON0bits.CHS == 0b0001)      // Canal 0001 -> 0010
            {
                ADCON0bits.CHS = 0b0010;
            }
            
            else if (ADCON0bits.CHS == 0b0010)      // Canal 0010 -> 0011
            {
                ADCON0bits.CHS = 0b0011;
            }
            
            else if (ADCON0bits.CHS == 0b0011)       // Canal 0011 -> 0000
            {
                ADCON0bits.CHS = 0b0000;
            }
            
            __delay_us(50);             
            ADCON0bits.GO = 1;          
        }
        
        // Comunicacion Maestro-Esclavo (SPI)
        PORTAbits.RA7 = 0;              // Activar Esclavo
        __delay_ms(10);                 // Delay
        
        SSPBUF = data_SPI;              // Transmitir datos
    }
    
    return;
}

void setup(void)                        // Configuración del PIC
{
    // I/O
    ANSEL = 0x0F;                       // Entradas analógicas
    ANSELH = 0;                         // I/O digitales
    
    // Oscilador
    OSCCONbits.IRCF = 0b0111;           // 4MHz
    OSCCONbits.SCS = 1;                 // Oscilador interno
    
    // Entradas
    TRISA = 0x0F;                       // PORTA Bits: |0|0|0|0|1|1|1|1|
    PORTA = 0;                          
    
    TRISB = 0x0F;                                        
    PORTB = 0;     
    
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 0;
    PORTC = 0;
    
    TRISD = 0x00;                          
    PORTD = 0;  
    
    TRISE = 0x00;
    PORTE = 0;
    
    // PORTB
    OPTION_REGbits.nRBPU = 0;           // Habilitar resistencias pull-up
    WPUBbits.WPUB0 = 1;                 // Resistencia pull-up en RB0
    WPUBbits.WPUB1 = 1;                 // Resistencia pull-up en RB1
    WPUBbits.WPUB2 = 1;                 // Resistencia pull-up en RB2
    WPUBbits.WPUB3 = 1;                 // Resistencia pull-up en RB3
    
    // ADC & PWMs
    ADCON0bits.ADCS = 0b01;             // Reloj de conversión: Fosc/8
    ADCON0bits.CHS = 0b0000;            // Canal para pin AN0
    ADCON1bits.VCFG0 = 0;               // Ref: VDD
    ADCON1bits.VCFG1 = 0;               // Ref: VSS
    ADCON1bits.ADFM = 0;                // Justificado a la izquierda
    ADCON0bits.ADON = 1;                // Habilitar ADC

    TRISCbits.TRISC2 = 1;               // Deshabilitar salida en CCP1
    CCP1CONbits.P1M = 0;                // Modo Single Output
    CCP1CONbits.CCP1M = 0b1100;         // PWM 1
    CCP2CONbits.CCP2M = 0b1111;         // PWM 2 
    
    CCPR1L = 0x0F;                    
    CCP1CONbits.DC1B = 0;     
    CCPR2L = 0x0F;
    PR2 = 250;               
    
    T2CONbits.T2CKPS = 0b11;
    T2CONbits.TMR2ON = 1;
    PIR1bits.TMR2IF = 0;

    while(PIR1bits.TMR2IF == 0);
        PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;               // Habilitar salida de PWM
    
    // TMR0
    OPTION_REGbits.T0CS = 0;            // Reloj interno para el TMR0
    OPTION_REGbits.T0SE = 0;            // Flanco de reloj ascendente
    OPTION_REGbits.PS2 = 1;             // 
    OPTION_REGbits.PS1 = 1;             //    -> Prescaler: 1:256
    OPTION_REGbits.PS0 = 1;             //
    
    // Comunicación serial
    TXSTAbits.SYNC = 0;                 // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;                 // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;              // 16-bits para generar el baud rate
    
    SPBRG = 207;                        // Baud rate: 9600
    SPBRGH = 0;                         //
    
    RCSTAbits.SPEN = 1;                 // Habilitar comunicación
    TXSTAbits.TX9 = 0;                  // Utilizar solo 8 bits
    TXSTAbits.TXEN = 1;                 // Habilitar transmisor
    RCSTAbits.CREN = 1;                 // Habilitar receptor
    
    // SPI
    SSPCONbits.SSPM = 0b0000;           // SPI: Master mode, Clock: Fosc/4
    SSPCONbits.CKP = 0;                 // Reloj inactivo al inicio
    SSPCONbits.SSPEN = 1;               // Habilitar pines del SPI
    SSPSTATbits.CKE = 1;                // Transmitir en flanco positivo
    SSPSTATbits.SMP = 1;                // Enviar al final del pulso de reloj
    
    // Interrupciones
    INTCONbits.GIE = 1;                 // Globales
    INTCONbits.PEIE = 1;                // Perifericas
    
    PIE1bits.ADIE = 1;                  // ADC
    PIR1bits.ADIF = 0;                  // Bandera ADC
    
    INTCONbits.T0IF = 0;                // TMR0
    INTCONbits.T0IE = 1;                // Bandera TMR0
    
    PIE1bits.SSPIE = 1;                 // SPI
    PIR1bits.SSPIF = 0;                 // Bandera SPI
    
    INTCONbits.RBIE = 1;                // PORTB
    INTCONbits.RBIF = 0;                // Bandera PORTB    
    IOCBbits.IOCB0 = 1;                 // On-change RB0
    IOCBbits.IOCB1 = 1;                 // On-change RB1
    IOCBbits.IOCB2 = 1;                 // On-change RB2
    IOCBbits.IOCB3 = 1;                 // On-change RB3
    
    PIE1bits.RCIE = 1;                  // Recepcion de datos
    
    // Valores iniciales
    PORTAbits.RA4 = 1;                  // Indicador Maestro
    SSPBUF = 0x00;                      // Dato inicial a transmitir
}

void reset_TMR0(void)                   // Resetear TMR0
{
    TMR0 = 0;                           // Prescaler
    INTCONbits.T0IF = 0;                // Limpiar bandera de interrupcion
    return;
}

void write_EEPROM(uint8_t address, uint8_t data)
{
    EEADR = address;
    EEDAT = data;
    
    INTCONbits.GIE = 0;             // Deshabilitar interrupciones
    
    EECON1bits.EEPGD = 0;           // Escribir EEPROM
    EECON1bits.WREN = 1;            // Habilitar escritura EEPROM
    
    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;              // Iniciar escritura
    
    while(PIR2bits.EEIF == 0);
    PIR2bits.EEIF = 0;
    
    EECON1bits.WREN = 0;            // Deshabilitar escritura EEPROM
    
    INTCONbits.RBIF = 0;            // Limpiar bandera de interrupcion
    INTCONbits.GIE = 1;             // Habilitar interrupciones
    
    return;
}

uint8_t read_EEPROM(uint8_t address)
{
    EEADR = address;
    
    EECON1bits.EEPGD = 0;           // Lectura EEPROM
    EECON1bits.RD = 1;              // Obtener dato de la EEPROM
    
    return EEDAT;                   // Retornar valor leido
}

void print(char str[])
{   
    uint8_t index = 0;
    
    while (str[index]!= '\0')
    {
        if (PIR1bits.TXIF)
        {             
            TXREG = str[index];    
            index++;                   
        }
    }
}

void run_EUSART(void)
{
    print("\r Menu principal: \r");
    print("1. Control manual de servos\r");
    print("2. Acceder a la EEPROM\r");
    print("3. Salir de la terminal\r");
    
    while(!PIR1bits.RCIF);                  // Recibir dato
    
    switch (read_serial)
    {
        case '1':
            print("\r Seleccionar la posicion del servo a modificar: \r");
            print("1. Izquierda\r");
            print("2. Centro\r");
            print("3. Derecha\r");
            
            while(!PIR1bits.RCIF);
            
            switch (read_serial)
            {
                case '1':
                    position_servo = (250 >> 1) + 125;
                    break;
                    
                case '2':
                    position_servo = (0 >> 1) + 125;
                    break;
                    
                case '3':
                    position_servo = (127 >> 1) + 125;
                    break;
            }
            
            print("\r Seleccionar servo a modificar (1-4): \r");
            
            while(!PIR1bits.RCIF);
            
            switch (read_serial)
            {
                case '1':                           // Servo 1
                    data_SPI = position_servo;
                    PORTDbits.RD0 = 1;
                    PORTDbits.RD0 = 0;
                    SSPBUF = data_SPI;
                    break;
                    
                case '2':                           // Servo 2
                    data_SPI = position_servo;
                    PORTDbits.RD0 = 0;
                    PORTDbits.RD0 = 1;
                    SSPBUF = data_SPI;
                    break;
                    
                case '3':                           // Servo 3
                    CCPR1L = (position_servo>>1)+123;
                    CCP1CONbits.DC1B = (position_servo & 0b01);
                    CCP1CONbits.DC1B0 = (position_servo>>7);
                    break;
                    
                case '4':                           // Servo 4
                    CCPR2L = (position_servo>>1)+123;
                    CCP1CONbits.DC1B = (position_servo & 0b01);
                    CCP1CONbits.DC1B0 = (position_servo>>7);
                    break;
            }
            
            break;
            
        case '2':
            print("\r Escribir/Leer en la EEPROM (1 o 2, respectivamente): \r");
            
            while(!PIR1bits.RCIF);
            
            switch (read_serial)
            {
                case '1':
                    print("\r Ingresar direccion a leer (Hex/Bin): \r");
                    
                    while(!PIR1bits.RCIF);
                    
                    
            }
            
            break;
            
        case '3':
            flag_EUSART = 0;
            print("\r Cerrando terminal... \r");
            PORTAbits.RA5 = flag_EUSART;
            break;
    }
    
    return;
}