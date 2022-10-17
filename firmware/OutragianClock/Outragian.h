/***
        /$$$$$$              /$$                                  /$$
       /$$__  $$            | $$                                 |__/
      | $$  \ $$ /$$   /$$ /$$$$$$    /$$$$$$  /$$$$$$   /$$$$$$  /$$  /$$$$$$  /$$$$$$$
      | $$  | $$| $$  | $$|_  $$_/   /$$__  $$|____  $$ /$$__  $$| $$ |____  $$| $$__  $$
      | $$  | $$| $$  | $$  | $$    | $$  \__/ /$$$$$$$| $$  \ $$| $$  /$$$$$$$| $$  \ $$
      | $$  | $$| $$  | $$  | $$ /$$| $$      /$$__  $$| $$  | $$| $$ /$$__  $$| $$  | $$
      |  $$$$$$/|  $$$$$$/  |  $$$$/| $$     |  $$$$$$$|  $$$$$$$| $$|  $$$$$$$| $$  | $$
       \______/  \______/    \___/  |__/      \_______/ \____  $$|__/ \_______/|__/  |__/
                                                        /$$  \ $$
                                                       |  $$$$$$/
                                                        \______/
        /$$$$$$  /$$                     /$$
       /$$__  $$| $$                    | $$
      | $$  \__/| $$  /$$$$$$   /$$$$$$$| $$   /$$
      | $$      | $$ /$$__  $$ /$$_____/| $$  /$$/
      | $$      | $$| $$  \ $$| $$      | $$$$$$/
      | $$    $$| $$| $$  | $$| $$      | $$_  $$
      |  $$$$$$/| $$|  $$$$$$/|  $$$$$$$| $$ \  $$
       \______/ |__/ \______/  \_______/|__/  \__/

      Danjovic - June 2022
      https://hackaday.io/project/186065-outragian-clock
*/

#ifndef _OUTRAGIAN_H_
#define _OUTRAGIAN_H_

// 16 segment display stuff
#define CA  // Common Anode

// segment position on the 16 bit shift register
// inherited from paganini clock, can be arbitrary

#define _A1   (1<<1 )
#define _A2   (1<<5 )
#define _B    (1<<6 )
#define _C    (1<<11)
#define _D1   (1<<9 )
#define _D2   (1<<13)
#define _E    (1<<14)
#define _F    (1<<3 )
#define _G1   (1<<15)
#define _G2   (1<<7 )
#define _H    (1<<2 )
#define _I    (1<<0 )
#define _J    (1<<4 )
#define _K    (1<<12)
#define _L    (1<<8 )
#define _M    (1<<10)



// Upcase letters
#define _HOUR_    (        _B|_C        |_E|_F|_G1|_G2                  )  // H
#define _MINUTE_  (        _B|_C        |_E|_F        |_H   |_J         )  // M


// other patterns
#define _BLANK_ 0 
#define _FULL_  (_A1|_A2|_B|_C|_D1|_D2|_E|_F|_G1|_G2|_H|_I|_J|_K|_L|_M)
#define _STAR_  (                            _G1|_G2|_H|_I|_J|_K|_L|_M)  


// 7 segment digits on the rightmost part of the 16 segment display 
#define _00_ (    _A2|_B|_C|_D1                     |_I      |_L   )
#define _01_ (        _B|_C                                        )
#define _02_ (    _A2|_B   |_D1              |_G2            |_L   )
#define _03_ (    _A2|_B|_C|_D1              |_G2                  )
#define _04_ (        _B|_C                  |_G2   |_I            )
#define _05_ (    _A2|   _C|_D1              |_G2   |_I            )
#define _06_ (    _A2|   _C|_D1              |_G2   |_I      |_L   )
#define _07_ (    _A2|_B|_C                                        )
#define _08_ (    _A2|_B|_C|_D1              |_G2   |_I      |_L   )
#define _09_ (    _A2|_B|_C|_D1              |_G2   |_I            )

// 7 segment digits on the leftmost part of the 16 segment display 
#define _10_  (                                       _I      |_L   )
#define _20_  (_A1              |_D2|_E   |_G1       |_I            )
#define _30_  (_A1              |_D2      |_G1       |_I      |_L   )
#define _40_  (                         _F|_G1       |_I      |_L   )
#define _50_  (_A1              |_D2   |_F|_G1                |_L   )
#define _60_  (_A1              |_D2|_E|_F|_G1                |_L   )
#define _70_  (_A1                                   |_I      |_L   )
#define _80_  (_A1              |_D2|_E|_F|_G1       |_I      |_L   )
#define _90_  (_A1              |_D2   |_F|_G1       |_I      |_L   )
#define _100_ (_A1              |_D2|_E|_F           |_I      |_L   )



// 
// Pin and Port definition
// As the display is positioned upside down on the board, the bit segmets were exchanged

// Port definition          // Phys     logical
#define _D1_PORT   PORTB    //  A1  ->   D1  
#define _D2_PORT   PORTC    //  A2  ->   D2  
#define _E_PORT    PORTC    //  B   ->    E  
#define _F_PORT    PORTD    //  C   ->    F  
#define _A1_PORT   PORTD    //  D1  ->   A1  
#define _A2_PORT   PORTD    //  D2  ->   A2  
#define _B_PORT    PORTD    //  E   ->    B  
#define _C_PORT    PORTB    //  F   ->    C  
#define _G2_PORT   PORTD    //  G1  ->    G1 
#define _G1_PORT   PORTC    //  G2  ->    G2 
#define _M_PORT    PORTB    //  H   ->    K  
#define _L_PORT    PORTB    //  I   ->    L  
#define _K_PORT    PORTC    //  J   ->    M  
#define _H_PORT    PORTD    //  K   ->    J  
#define _I_PORT    PORTD    //  L   ->    I  
#define _J_PORT    PORTD    //  M   ->    H  
#define _DP_PORT   PORTB

// bit Definition
#define _D1_BIT    2  
#define _D2_BIT    0  
#define _E_BIT     2  
#define _F_BIT     1 
#define _A1_BIT    2  
#define _A2_BIT    5  
#define _B_BIT     6  
#define _C_BIT     0 
#define _G2_BIT    7  
#define _G1_BIT    3  
#define _M_BIT     1  
#define _L_BIT     4  
#define _K_BIT     1 
#define _H_BIT     0  
#define _I_BIT     3  
#define _J_BIT     4
#define _DP_BIT    5


// segment position on the 16 bit shift register (inherited from paganini clock, can be arbitrary
#define _A1   (1<<1 )
#define _A2   (1<<5 )
#define _B    (1<<6 )
#define _C    (1<<11)
#define _D1   (1<<9 )
#define _D2   (1<<13)
#define _E    (1<<14)
#define _F    (1<<3 )
#define _G1   (1<<15)
#define _G2   (1<<7 )
#define _H    (1<<2 )
#define _I    (1<<0 )
#define _J    (1<<4 )
#define _K    (1<<12)
#define _L    (1<<8 )
#define _M    (1<<10)


#if defined CA
#define _ON 0
#define _OFF 1

// Set LEDS
#define  CLEAR_A1   _A1_PORT  |= (1 <<  _A1_BIT )
#define  CLEAR_A2   _A2_PORT  |= (1 <<  _A2_BIT )
#define  CLEAR_B    _B_PORT   |= (1 <<  _B_BIT  )
#define  CLEAR_C    _C_PORT   |= (1 <<  _C_BIT  )
#define  CLEAR_D1   _D1_PORT  |= (1 <<  _D1_BIT )
#define  CLEAR_D2   _D2_PORT  |= (1 <<  _D2_BIT )
#define  CLEAR_E    _E_PORT   |= (1 <<  _E_BIT  )
#define  CLEAR_F    _F_PORT   |= (1 <<  _F_BIT  )
#define  CLEAR_G1   _G1_PORT  |= (1 <<  _G1_BIT )
#define  CLEAR_G2   _G2_PORT  |= (1 <<  _G2_BIT )
#define  CLEAR_H    _H_PORT   |= (1 <<  _H_BIT  )
#define  CLEAR_I    _I_PORT   |= (1 <<  _I_BIT  )
#define  CLEAR_J    _J_PORT   |= (1 <<  _J_BIT  )
#define  CLEAR_K    _K_PORT   |= (1 <<  _K_BIT  )
#define  CLEAR_L    _L_PORT   |= (1 <<  _L_BIT  )
#define  CLEAR_M    _M_PORT   |= (1 <<  _M_BIT  )
#define  CLEAR_DP   _DP_PORT  |= (1 <<  _DP_BIT )

// Clear LEDs
#define  SET_A1   _A1_PORT  &= ~(1 <<  _A1_BIT )
#define  SET_A2   _A2_PORT  &= ~(1 <<  _A2_BIT )
#define  SET_B    _B_PORT   &= ~(1 <<  _B_BIT  )
#define  SET_C    _C_PORT   &= ~(1 <<  _C_BIT  )
#define  SET_D1   _D1_PORT  &= ~(1 <<  _D1_BIT )
#define  SET_D2   _D2_PORT  &= ~(1 <<  _D2_BIT )
#define  SET_E    _E_PORT   &= ~(1 <<  _E_BIT  )
#define  SET_F    _F_PORT   &= ~(1 <<  _F_BIT  )
#define  SET_G1   _G1_PORT  &= ~(1 <<  _G1_BIT )
#define  SET_G2   _G2_PORT  &= ~(1 <<  _G2_BIT )
#define  SET_H    _H_PORT   &= ~(1 <<  _H_BIT  )
#define  SET_I    _I_PORT   &= ~(1 <<  _I_BIT  )
#define  SET_J    _J_PORT   &= ~(1 <<  _J_BIT  )
#define  SET_K    _K_PORT   &= ~(1 <<  _K_BIT  )
#define  SET_L    _L_PORT   &= ~(1 <<  _L_BIT  )
#define  SET_M    _M_PORT   &= ~(1 <<  _M_BIT  )
#define  SET_DP   _DP_PORT  &= ~(1 <<  _DP_BIT )

#elif defined CC
#define _ON 1
#define _OFF 0
// Set LEDS
#define  SET_A1   _A1_PORT  |= (1 <<  _A1_BIT )
#define  SET_A2   _A2_PORT  |= (1 <<  _A2_BIT )
#define  SET_B    _B_PORT   |= (1 <<  _B_BIT  )
#define  SET_C    _C_PORT   |= (1 <<  _C_BIT  )
#define  SET_D1   _D1_PORT  |= (1 <<  _D1_BIT )
#define  SET_D2   _D2_PORT  |= (1 <<  _D2_BIT )
#define  SET_E    _E_PORT   |= (1 <<  _E_BIT  )
#define  SET_F    _F_PORT   |= (1 <<  _F_BIT  )
#define  SET_G1   _G1_PORT  |= (1 <<  _G1_BIT )
#define  SET_G2   _G2_PORT  |= (1 <<  _G2_BIT )
#define  SET_H    _H_PORT   |= (1 <<  _H_BIT  )
#define  SET_I    _I_PORT   |= (1 <<  _I_BIT  )
#define  SET_J    _J_PORT   |= (1 <<  _J_BIT  )
#define  SET_K    _K_PORT   |= (1 <<  _K_BIT  )
#define  SET_L    _L_PORT   |= (1 <<  _L_BIT  )
#define  SET_M    _M_PORT   |= (1 <<  _M_BIT  )
#define  SET_DP   _DP_PORT  |= (1 <<  _DP_BIT )

// Clear LEDs
#define  CLEAR_A1   _A1_PORT  &= ~(1 <<  _A1_BIT )
#define  CLEAR_A2   _A2_PORT  &= ~(1 <<  _A2_BIT )
#define  CLEAR_B    _B_PORT   &= ~(1 <<  _B_BIT  )
#define  CLEAR_C    _C_PORT   &= ~(1 <<  _C_BIT  )
#define  CLEAR_D1   _D1_PORT  &= ~(1 <<  _D1_BIT )
#define  CLEAR_D2   _D2_PORT  &= ~(1 <<  _D2_BIT )
#define  CLEAR_E    _E_PORT   &= ~(1 <<  _E_BIT  )
#define  CLEAR_F    _F_PORT   &= ~(1 <<  _F_BIT  )
#define  CLEAR_G1   _G1_PORT  &= ~(1 <<  _G1_BIT )
#define  CLEAR_G2   _G2_PORT  &= ~(1 <<  _G2_BIT )
#define  CLEAR_H    _H_PORT   &= ~(1 <<  _H_BIT  )
#define  CLEAR_I    _I_PORT   &= ~(1 <<  _I_BIT  )
#define  CLEAR_J    _J_PORT   &= ~(1 <<  _J_BIT  )
#define  CLEAR_K    _K_PORT   &= ~(1 <<  _K_BIT  )
#define  CLEAR_L    _L_PORT   &= ~(1 <<  _L_BIT  )
#define  CLEAR_M    _M_PORT   &= ~(1 <<  _M_BIT  )
#define  CLEAR_DP   _DP_PORT  &= ~(1 <<  _DP_BIT )

#else
#error Must Define CA or CC
#endif




// I2C stuff
#define PORT_I2C    PORTC
#define DDR_I2C     DDRC
#define PIN_I2C     PINC
#define bSCL        5
#define bSDA        4
#define sendACK     true
#define sendNACK    false
#define RTC_ADDRESS 0x68



#endif

//     __ _      _    
//    / _(_)_ _ (_)___
//   |  _| | ' \| (_-<
//   |_| |_|_||_|_/__/
//
