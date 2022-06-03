/*
 * File:            main.c
 * Author:          Andrea Barrientos Pineda, 20575
 *
 * Creado:          23/05/2022
 * Modificado:      02/06/2022
 * 
 * Proyecto 2       Programa PIC Slave - Salidas de datos
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

//-------------------- Declaración de funciones --------------------------------
//------------------------------------------------------------------------------
void setup(void);                      // Configuración del PIC
void reset_TMR0(void);                 // Resetear TMR0

//-------------------- Interrupciones ------------------------------------------
//------------------------------------------------------------------------------
void __interrupt() isr (void)           
{
    // Interrupcion ADC
    if (PIR1bits.ADIF)                  
    {   
        PIR1bits.ADIF = 0;              // Desactivar interrupcion
    }
    
    // Interrupcion TMR0
    if (INTCONbits.T0IF)                
    {
        reset_TMR0();                   // Resetear TMR0
        INTCONbits.T0IF = 0;            // Desactivar interrupcion
    }
    
    // Interrupcion SPI
    if (PIR1bits.SSPIF)
    {
        PIR1bits.SSPIF = 0;             // Desactivar interrupcion
    }
}

//-------------------- Programa principal --------------------------------------
//------------------------------------------------------------------------------

void main(void)                         
{
    setup();                            
    reset_TMR0();                       
    __delay_us(50);
    
    while (1)                         
    {
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
        while(!SSPSTATbits.BF);         // Esperar lectura de datos
        data_SPI = SSPBUF;              // Almacenar datos recibidos
        
        // Servo 1
        if (PORTBbits.RB0)
        {
            CCPR1L = (data_SPI>>1)+123;
            CCP1CONbits.DC1B = (data_SPI & 0b01);
            CCP1CONbits.DC1B0 = (data_SPI>>7);
        }
        
        // Servo 2
        else if (PORTBbits.RB1)
        {
            CCPR2L = (data_SPI>>1)+123;
            CCP1CONbits.DC1B = (data_SPI & 0b01);
            CCP1CONbits.DC1B0 = (data_SPI>>7);
        }
    }
    
    return;
}

void setup(void)                        // Configuración del PIC
{
    // I/O
    ANSEL = 0x0F;                       // Entradas analogicas
    ANSELH = 0;                         // I/O digitales
    
    // Oscilador
    OSCCONbits.IRCF = 0b0111;           // 4MHz
    OSCCONbits.SCS = 1;                 // Oscilador interno
    
    // Entradas
    TRISA = 0x23;                       // PORTA Bits: |0|0|1|0|0|0|1|1|
    PORTA = 0;                          
    
    TRISB = 0x03;                       // PORTC Bits: |0|0|0|0|0|0|1|1|                 
    PORTB = 0;     
    
    TRISC = 0x18;                       // PORTC Bits: |0|0|0|1|1|0|0|0|
    PORTC = 0;
    
    TRISD = 0x00;                          
    PORTD = 0;       
    
    TRISE = 0x00;
    PORTE = 0;
    
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
        TRISCbits.TRISC1 = 0;
        TRISCbits.TRISC2 = 0;
    
    TRISCbits.TRISC2 = 0;               // Habilitar salida de PWM
    
    // TMR0
    OPTION_REGbits.T0CS = 0;            // Reloj interno para el TMR0
    OPTION_REGbits.T0SE = 0;            // Flanco de reloj ascendente
    OPTION_REGbits.PS2 = 1;             // 
    OPTION_REGbits.PS1 = 1;             //    -> Prescaler: 1:256
    OPTION_REGbits.PS0 = 1;             //
    
    // SPI
    SSPCONbits.SSPM = 0b0100;           // SPI: Slave mode, Clock: Fosc/4
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
    
    // Valores iniciales
    PORTBbits.RB0 = 1;                  // Indicador esclavo
}

void reset_TMR0(void)                   // Resetear TMR0
{
    TMR0 = 0;                           // Prescaler
    INTCONbits.T0IF = 0;                // Limpiar bandera de interrupcion
    return;
}