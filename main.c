#include <xc.h>
#include "test.h"

/**
 * Bits de configuration:
 */
#pragma config FOSC = INTIO67   // Osc. interne, A6 et A7 comme IO.
#pragma config IESO = OFF       // Pas d'osc. au démarrage.
#pragma config FCMEN = OFF      // Pas de monitorage de l'oscillateur.

// Nécessaires pour ICSP / ICD:
#pragma config MCLRE = EXTMCLR  // RE3 est actif comme master reset.
#pragma config WDTEN = OFF      // Watchdog inactif.
#pragma config LVP = OFF        // Single Supply Enable bits off.

typedef enum {
    AVANT = 0b01,
    ARRIERE = 0b10
} Direction;

/**
 * Indique la direction correspondante à la valeur du potentiomètre.
 * @param v Valeur du potentiomètre.
 * @return AVANT ou ARRIERE.
 */
Direction conversionDirection(unsigned char v) {
    if (v<128){
       return ARRIERE; 
    }
    return AVANT;
}

/**
 * Indique le cycle de travail PWM correspondant à la valeur du potentiomètre.
 * @param v Valeur du potentiomètre.
 * @return Cycle de travail du PWM.
 */
unsigned char conversionMagnitude(unsigned char v) {
    if (v<128){
        return 254-2*v;
    }
    
    return 2*v-256;
}

#ifndef TEST

/**
 * Initialise le hardware.
 */
static void hardwareInitialise() {
    //Mise du Fosc à 1MHz par le clk interne
    OSCCONbits.IRCF=3; // Permet de seter INTosc à 1 MHz
    OSCCONbits.SCS=2; // Choix sur de l'oscillateur sur Clock MUX
    
    // Prépare Temporisateur 2 pour PWM (compte jusqu'à 255 en 1ms):
    T2CONbits.T2CKPS=0; //Division par 1 de Fosc/4
    PR2=255; //Période en "pas"
    T2CONbits.T2OUTPS=9; //Interruption toutes les 10ms
    PIE1bits.TMR2IE=1; //Autorise les interruptions du Timer 2
    T2CONbits.TMR2ON=1; //Enclencher le timer 2
    
    // Configure PWM 1 pour émettre un signal de 1KHz:
    CCPTMRS0bits.C1TSEL=0; //Aiguille Timer 2 sur CCP1
    CCP1CONbits.P1M=0; //Configuration générateur PWM
    CCP1CONbits.CCP1M=0b1100; //Est-ce nécessaire
    TRISCbits.RC2=0; //Pin RC2 mise en sortie
    
    
    

    // Configure RC0 et RC1 pour gérer la direction du moteur:
    TRISCbits.RC0=0; //Pin RC0 mise en sortie
    TRISCbits.RC1=0; //Pin RC1 mise en sortie
    
    // Active le module de conversion A/D:
    TRISBbits.RB3=1; //Pin RB3 en entrée
    ADCON0bits.CHS=0b1001; // active AN9
    ADCON1bits.PVCFG=0; // Vref+ en interne sur VDD
    ADCON1bits.NVCFG=0; // Vref- sur Vss (0V)
    ADCON2bits.ADFM=0; // Justifié à gauche (Adresse H)
    PIE1bits.ADIE=1; // Active l'interruption "fin de conversion AD"
    ADCON0bits.ADON=1; // Active l'entrée

    // Active les interruptions générales:
    RCONbits.IPEN=1; //Gère les alarmes en low priority
    INTCONbits.PEIE=1; //autorise les interruptions périphériques
    INTCONbits.GIE=1; //autorise les interruptions
}

/**
 * Point d'entrée des interruptions.
 */
void low_priority interrupt interruptionsBassePriorite() {
    if (PIR1bits.TMR2IF) {
        PIR1bits.TMR2IF = 0;
        ADCON0bits.GO = 1;
    }
    
    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0;
        PORTC = conversionDirection(ADRESH);
        CCPR1L = conversionMagnitude(ADRESH);
    }
}

/**
 * Point d'entrée pour l'émetteur de radio contrôle.
 */
void main(void) {
    hardwareInitialise();

    while(1);
}
#endif

#ifdef TEST
void testConversionMagnitude() {
    testeEgaliteEntiers("CM01", conversionMagnitude(0),   254);
    testeEgaliteEntiers("CM02", conversionMagnitude(1),   252);
    testeEgaliteEntiers("CM03", conversionMagnitude(2),   250);
    
    testeEgaliteEntiers("CM04", conversionMagnitude(125),   4);
    testeEgaliteEntiers("CM05", conversionMagnitude(126),   2);
    
    testeEgaliteEntiers("CM06", conversionMagnitude(127),   0);
    testeEgaliteEntiers("CM07", conversionMagnitude(128),   0);

    testeEgaliteEntiers("CM08", conversionMagnitude(129),   2);
    testeEgaliteEntiers("CM09", conversionMagnitude(130),   4);
    
    testeEgaliteEntiers("CM10", conversionMagnitude(253), 250);
    testeEgaliteEntiers("CM11", conversionMagnitude(254), 252);
    testeEgaliteEntiers("CM12", conversionMagnitude(255), 254);
}
void testConversionDirection() {
    testeEgaliteEntiers("CD01", conversionDirection(  0), ARRIERE);    
    testeEgaliteEntiers("CD02", conversionDirection(  1), ARRIERE);    
    testeEgaliteEntiers("CD03", conversionDirection(127), ARRIERE);    
    testeEgaliteEntiers("CD04", conversionDirection(128), AVANT);
    testeEgaliteEntiers("CD05", conversionDirection(129), AVANT);
    testeEgaliteEntiers("CD06", conversionDirection(255), AVANT);    
}
void main() {
    initialiseTests();
    testConversionMagnitude();
    testConversionDirection();
    finaliseTests();
    while(1);
}
#endif
