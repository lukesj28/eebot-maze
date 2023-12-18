;****************************************************************************
;* lab 7 I.e project*
;****************************************************************************
; export symbols
              XDEF Entry, _Startup ; export ‘Entry’ symbol
              ABSENTRY Entry ; for absolute assembly: mark
              INCLUDE "derivative.inc"
;***************************************************************************************************
; equates section
;***************************************************************************************************

; Other codes
NULL          EQU   00                    ; The string ’null terminator’
CR            EQU   $0D                   ; ’Carriage Return’ character
SPACE         EQU   ' '                   ; The ’space’ character

; Turning Timers
T_L           EQU   6
T_R           EQU   6

; LCD Addresses
LCD_CNTR      EQU   PTJ                   ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT       EQU   PORTB                 ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E         EQU   $80                   ; LCD E-signal pin
LCD_RS        EQU   $40                   ; LCD RS-signal pin

; States for robot
START         EQU   0                     ;Start State
FWD           EQU   1                     ;Forward State
ALL_STP       EQU   2                     ;All Stop State
L_TRN         EQU   3                     ;Left Turn State
R_TRN         EQU   4                     ;Right Turn State
RV_TRN        EQU   5                     ;Reverse Turn State
L_ALIGN       EQU   6                     ; L_ALIGN & R_ALIGN are the states that allow
R_ALIGN       EQU   7                     ; the robot to align and correctly straighten out

; Liquid Crystal Display Equates
CLEAR_HOME    EQU   $01                   ; Clear the display and home the cursor
INTERFACE     EQU   $38                   ; 8 bit interface, two line display
CURSOR_OFF    EQU   $0C                   ; Display on, cursor off
SHIFT_OFF     EQU   $06                   ; Address increments, no character shift
LCD_SEC_LINE  EQU   64                    ; Starting addr. of 2nd line of LCD (note decimal value!)

; variable/data section
              ORG   $3800
             
             
; Storage Registers (9S12C32 RAM space: $3800 ... $3FFF)
SENSOR_LINE   FCB   $01                     ; Storage for guider sensor readings
SENSOR_BOW    FCB   $23                     ; Initialized to test values
SENSOR_PORT   FCB   $45
SENSOR_MID    FCB   $67
SENSOR_STBD   FCB   $89
SENSOR_NUM    RMB   1                       ; The currently selected sensor

; Initial values based on the initial readings & variance
B_LINE     FCB   $9D                       ;specific eebot version addresses
B_BOW      FCB   $CA
B_MID      FCB   $CA
B_PORT     FCB   $CA
B_STBD     FCB   $CA

; Variance initialization  found by testing that allow the robot to turn 180                                        
V_LINE      FCB   $18                    
V_BOW       FCB   $30                    
V_PORT      FCB   $20                    
V_MID       FCB   $20                    
V_STBD      FCB   $15

; TOP and BOTTOM Line display
TOP_LINE      RMB   20                    
              FCB   NULL                              
BOT_LINE      RMB   20                    
              FCB   NULL                              
CLEAR_LINE    FCC   '                  '  ; Clear the line of display
              FCB   NULL                  ; terminated by null
             
TEMP          RMB   1                     ; Temporary location

; variable section//
              ORG   $3900                  
TOF_COUNTER   dc.b  0                      
CRNT_STATE    dc.b  2                       ; crnt register
T_TURN        ds.b  1                       ; time
NO_BLANK      ds.b  1                       ; Used in ’leading zero’ blanking by BCD2ASC
BCD_SPARE     RMB   2
TEN_THOUS     ds.b  1                       ; 10,000 digit
THOUSANDS     ds.b  1                       ; 1,000 digit
HUNDREDS      ds.b  1                       ; 100 digit
TENS          ds.b  1                       ; 10 digit
UNITS         ds.b  1                       ; 1 digit









; code section
              ORG   $4000 ; Start of program text (FLASH memory)
Entry:                                                                      
_Startup:

;Initialization

              LDS   #$4000                 ; Initialize the stack pointer
              CLI                          ; Enable interrupts
              JSR   INIT                   ; Initialize ports
              JSR   openADC                ; Initialize the ADC
              JSR   openLCD                ; Initialize the LCD
              JSR   CLR_LCD_BUF            ; Write ’space’ characters to the LCD buffer    
;// added code                
              BSET  DDRA,%00000011         ; port dir                      
              BSET  DDRT,%00110000         ; speed                                                                                              
              JSR   initAD                                                    
              JSR   openLCD                                    
              JSR   clrLCD                 ; Clear LCD                                                                                
              LDX   #msg1                                              
              JSR   putsLCD                                                        
              LDAA  #$8A                   ; Move LCD cursor to the second row          
              JSR   cmd2LCD                                                          
              LDX   #msg2                                              
              JSR   putsLCD
              LDAA   #$C0
              JSR   cmd2LCD                    
              LDX   #msg3                                            
              JSR   putsLCD                                                                                  
              LDAA  #$C7                   ; Move LCD cursor to the second row          
              JSR   cmd2LCD                                                        
              LDX   #msg4                                              
              JSR   putsLCD                            
              JSR   ENABLE_TOF             ; Jump to TOF initialization
;---------------------------------------------------------------------------
; Display Sensors
MAIN        
              JSR   G_LEDS_ON           ; Enable the guider LEDs  
              JSR   READ_SENSORS         ; Read the 5 guider sensors  
              JSR   G_LEDS_OFF            ; Disable the guider LEDs                        
              JSR   UPDT_DISPL      
              LDAA  CRNT_STATE        
              JSR   DISPATCHER      
              BRA   MAIN                    
; data section
msg1          dc.b  "S: ",0         ;Current State
msg2          dc.b  "R: ",0         ;Sensor readings
msg3          dc.b  "V: ",0         ;Battery Voltage
msg4          dc.b  "B: ",0         ;Bumper Status
tab           dc.b  "START  ",0
              dc.b  "FWD    ",0
              dc.b  "ALL_STP",0
              dc.b  "LTURN  ",0
              dc.b  "RTURN  ",0
              dc.b  "REVTURN ",0
              dc.b  "LTimed ",0    
              dc.b  "RTimed ",0  
                 
; subroutine section
; Start of the Dispatcher                                                                  
;**********************************************************************
;* State Dispatcher
;*
;* This routine calls the appropriate state handler based on the current
;* state.
;* Input: Current state in ACCA
;* Returns: None
;* Clobbers: Everything
DISPATCHER       JSR   CHCK_STRT         ; Jump to the beginning of the state machine
                 RTS                    ; Return from the dispatcher
               
CHCK_STRT        CMPA  #START           ; Compare value with #START
                 BNE   CHCK_FWD         ; If not equal, check next condition (forward)
                 JSR   START_ST         ; Call subroutine for starting state
                 RTS                    ; Return from CHCK_STRT                                      

CHCK_FWD         CMPA  #FWD             ; Compare value with #FWD
                 BNE   CHCK_STP         ; If not equal, check next condition (stop)
                 JSR   FWD_ST           ; Call subroutine for forward state
                 RTS                    ; Return from CHCK_FWD                                    

CHCK_STP         CMPA  #ALL_STP         ; Compare value with #ALL_STP
                 BNE   CHCK_L_TRN       ; If not equal, check next condition (left turn)
                 JSR   ALL_STP_ST       ; Call subroutine for stop state
                 RTS                    ; Return from CHCK_STP                                    

CHCK_L_TRN       CMPA  #L_TRN           ; Compare value with #L_TRN
                 BNE   CHCK_R_TRN       ; If not equal, check next condition (right turn)
                 JSR   FULL_L           ; Call subroutine for left turn
                 RTS                    ; Return from CHCK_L_TRN                                    

CHCK_R_TRN       CMPA  #R_TRN           ; Compare value with #R_TRN
                 BNE   CHCK_RV_TRN      ; If not equal, check next condition (reverse turn)
                 JSR   FULL_R           ; Call subroutine for right turn
                 RTS                    ; Return from CHCK_R_TRN                                  
                 
CHCK_RV_TRN      CMPA  #RV_TRN          ; Compare value with #RV_TRN
                 BNE   CHCK_L_ALIGN     ; If not equal, check next condition (left alignment)
                 JSR   REV_TURN_ST      ; Call subroutine for reverse turn
                 RTS                    ; Return from CHCK_RV_TRN

CHCK_L_ALIGN     CMPA  #L_ALIGN         ; Compare value with #L_ALIGN
                 BNE   CHCK_R_ALIGN     ; If not equal, check next condition (right alignment)
                 JSR   LEFT_ALIGN_EXIT  ; Call subroutine for left alignment exit
                 RTS                    ; Return from CHCK_L_ALIGN

CHCK_R_ALIGN     CMPA  #R_ALIGN         ; Compare value with #R_ALIGN
                 BNE   INVALID_STATE    ; If not equal, check next condition (invalid state)
                 JSR   RIGHT_ALIGN_EXIT ; Call subroutine for right alignment exit
                 RTS                    ; Return from CHCK_R_ALIGN                                    
                     
INVALID_STATE     SWI                                       ; Break monitor  
START_ST         BRCLR  PORTAD0, %00000100, NO_START   ; Check condition and jump if clear
                 JSR    INIT_FWD                       ; Initialize forward motion
                 MOVB   #FWD, CRNT_STATE              ; Set current state to forward                                                      
NO_START  RTS                                                                                        
FWD_ST            BRSET   PORTAD0, %00000100, NO_FWD_BUMP                                
                  MOVB    #RV_TRN, CRNT_STATE                                                      
                                                                                             
                  JSR     UPDT_DISPL                                                            
                                                                                                 
                  JSR     INIT_REV                                                                
                  LDY     #8000                                                                  
                  JSR     del_50us                                                              
                                                                                               
                  JSR     INIT_LEFT                                                              
                  LDY     #8000                                                                  
                  JSR     del_50us                                                                
                  LBRA    EXIT                                                                    
                                                                                                 
NO_FWD_BUMP       BRSET   PORTAD0, %00001000, NO_FWD_REAR_BUMP    
                  MOVB    #ALL_STP, CRNT_STATE                                                
                  JSR     INIT_STOP                                              
                  LBRA    EXIT                                                                    
                                                                                                 
;****************************************************************************                                                                                              
;This assembly code snippet involves some sensor-based conditional checks to determine the course of action based on the readings. Here's a breakdown:  
;****************************************************************************                                                                                                
                                                                                                 
NO_FWD_REAR_BUMP  LDAA    SENSOR_BOW      ; Load the value of SENSOR_BOW into the accumulator (A)
                  ADDA    V_BOW           ; Add the value of V_BOW to the accumulator (A)
                  CMPA    B_BOW           ; Compare the value in the accumulator (A) with B_BOW
                  BPL     NO_ALIGN        ; Branch if the result is greater than or equal to zero (No Alignment)

                  LDAA    SENSOR_MID      ; Load the value of SENSOR_MID into the accumulator (A)
                  ADDA    V_MID           ; Add the value of V_MID to the accumulator (A)
                  CMPA    B_MID           ; Compare the value in the accumulator (A) with B_MID
                  BPL     NO_ALIGN        ; Branch if the result is greater than or equal to zero (No Alignment)

                  LDAA    SENSOR_LINE     ; Load the value of SENSOR_LINE into the accumulator (A)
                  ADDA    V_LINE          ; Add the value of V_LINE to the accumulator (A)
                  CMPA    B_LINE          ; Compare the value in the accumulator (A) with B_LINE
                  BPL     GO_RIGHT_ALIGN  ; Branch if the result is greater than or equal to zero (Go Right Alignment)

                  LDAA    SENSOR_LINE     ; Load the value of SENSOR_LINE into the accumulator (A)
                  SUBA    V_LINE          ; Subtract the value of V_LINE from the accumulator (A)
                  CMPA    B_LINE          ; Compare the value in the accumulator (A) with B_LINE
                  BMI     GO_LEFT_ALIGN   ; Branch if the result is less than zero (Go Left Alignment)
                                                 
                                                                                                 
NO_ALIGN          LDAA    SENSOR_STBD      ; Load the value of SENSOR_PORT into the accumulator (A)
                  ADDA    V_STBD           ; Add the value of V_PORT to the accumulator (A)
                  CMPA    B_STBD           ; Compare the value in the accumulator (A) with B_PORT
                  BPL     PART_RIGHT_TURN   ; Branch if the result is greater than or equal to zero (Partial Left Turn)
                  BMI     NO_STBD_DET      ; Branch if the result is less than zero to NO_PORT_DET

NO_STBD_DET       LDAA    SENSOR_BOW       ; Load the value of SENSOR_BOW into the accumulator (A)
                  ADDA    V_BOW            ; Add the value of V_BOW to the accumulator (A)
                  CMPA    B_BOW            ; Compare the value in the accumulator (A) with B_BOW
                  BPL     EXIT             ; Branch if the result is greater than or equal to zero to EXIT
                  BMI     NO_BOW_DET       ; Branch if the result is less than zero to NO_BOW_DET

NO_BOW_DET        LDAA    SENSOR_PORT      ; Load the value of SENSOR_STBD into the accumulator (A)
                  ADDA    V_PORT           ; Add the value of V_STBD to the accumulator (A)
                  CMPA    B_PORT           ; Compare the value in the accumulator (A) with B_STBD
                  BPL     PART_LEFT_TURN  ; Branch if the result is greater than or equal to zero (Partial Right Turn)
                  BMI     EXIT             ; Branch if the result is less than zero to EXIT

PART_LEFT_TURN    LDY     #10000            ; Load Y register with the value 6000
                  JSR     del_50us         ; Jump to subroutine del_50us (delay of 50 microseconds)
                  JSR     INIT_LEFT        ; Jump to subroutine INIT_LEFT
                  MOVB    #L_TRN, CRNT_STATE ; Move the value L_TRN to the CRNT_STATE variable
                  LDY     #6000            ; Load Y register with the value 6000
                  JSR     del_50us         ; Jump to subroutine del_50us (delay of 50 microseconds)
                  BRA     EXIT             ; Unconditional branch to EXIT
                                                           
                                                                                                 
PART_RIGHT_TURN   LDY     #10000            ; Load Y register with the value 6000
                  jsr     del_50us         ; Jump to subroutine del_50us (delay of 50 microseconds)
                  JSR     INIT_RIGHT       ; Jump to subroutine INIT_RIGHT
                  MOVB    #R_TRN, CRNT_STATE ; Move the value R_TRN to the CRNT_STATE variable
                  LDY     #6000            ; Load Y register with the value 6000
                  JSR     del_50us         ; Jump to subroutine del_50us (delay of 50 microseconds)
                  BRA     EXIT             ; Unconditional branch to EXIT

GO_LEFT_ALIGN     JSR     INIT_LEFT        ; Jump to subroutine INIT_LEFT
                  MOVB    #L_ALIGN, CRNT_STATE ; Move the value L_ALIGN to the CRNT_STATE variable
                  BRA     EXIT             ; Unconditional branch to EXIT

GO_RIGHT_ALIGN    JSR     INIT_RIGHT       ; Jump to subroutine INIT_RIGHT
                  MOVB    #R_ALIGN, CRNT_STATE ; Move the value R_ALIGN to the CRNT_STATE variable
                  BRA     EXIT             ; Unconditional branch to EXIT

EXIT              RTS                     ; Return from subroutine

FULL_L            LDAA    SENSOR_BOW       ; Load the value of SENSOR_BOW into the accumulator (A)
                  ADDA    V_BOW            ; Add the value of V_BOW to the accumulator (A)
                  CMPA    B_BOW            ; Compare the value in the accumulator (A) with B_BOW
                  BPL     LEFT_ALIGN_EXIT  ; Branch if the result is greater than or equal to zero (Left Alignment Exit)
                  BMI     EXIT             ; Branch if the result is less than zero to EXIT

LEFT_ALIGN_EXIT   MOVB    #FWD, CRNT_STATE ; Move the value FWD to the CRNT_STATE variable
                  JSR     INIT_FWD         ; Jump to subroutine INIT_FWD
                  BRA     EXIT             ; Unconditional branch to EXIT

                                                                                     
FULL_R            LDAA    SENSOR_BOW       ; Load the value of SENSOR_BOW into the accumulator (A)
                  ADDA    V_BOW            ; Add the value of V_BOW to the accumulator (A)
                  CMPA    B_BOW            ; Compare the value in the accumulator (A) with B_BOW
                  BPL     RIGHT_ALIGN_EXIT ; Branch if the result is greater than or equal to zero (Right Alignment Exit)
                  BMI     EXIT             ; Branch if the result is less than zero to EXIT

RIGHT_ALIGN_EXIT  MOVB    #FWD, CRNT_STATE ; Move the value FWD to the CRNT_STATE variable
                  JSR     INIT_FWD         ; Jump to subroutine INIT_FWD
                  BRA     EXIT             ; Unconditional branch to EXIT

REV_TURN_ST       LDAA    SENSOR_BOW       ; Load the value of SENSOR_BOW into the accumulator (A)
                  ADDA    V_BOW            ; Add the value of V_BOW to the accumulator (A)
                  CMPA    B_BOW            ; Compare the value in the accumulator (A) with B_BOW
                  BMI     EXIT             ; Branch if the result is less than zero to EXIT

                  JSR     INIT_LEFT        ; Jump to subroutine INIT_LEFT
                  MOVB    #FWD, CRNT_STATE ; Move the value FWD to the CRNT_STATE variable
                  JSR     INIT_FWD         ; Jump to subroutine INIT_FWD
                  BRA     EXIT             ; Unconditional branch to EXIT

ALL_STP_ST        BRSET   PORTAD0, %00000100, NO_START_BUMP ; Branch if bit 2 of PORTAD0 is set to NO_START_BUMP label
                  MOVB    #START, CRNT_STATE               ; Move the value START to the CRNT_STATE variable
NO_START_BUMP     RTS                                     ; Return from subroutine
                                             


; Initialization Subroutines
;***************************************************************************************************
; Sets up for a right turn
INIT_RIGHT        BSET    PORTA,%00000010         ; Set right motor direction to reverse
                  BCLR    PORTA,%00000001         ; Set left motor direction to forward
                  LDAA    TOF_COUNTER             ; Load the TOF_COUNTER value
                  ADDA    #T_R                    ; Add a constant value T_R to accumulator A
                  STAA    T_TURN                  ; Store the result in T_TURN
                  RTS                             ; Return from the subroutine

; Sets up for a left turn                  
INIT_LEFT         BSET    PORTA,%00000001         ; Set left motor direction to reverse
                  BCLR    PORTA,%00000010         ; Set right motor direction to forward
                  LDAA    TOF_COUNTER             ; Load the TOF_COUNTER value
                  ADDA    #T_L                    ; Add a constant value T_L to accumulator A
                  STAA    T_TURN                  ; Store the result in T_TURN
                  RTS                             ; Return from the subroutine

; Sets up for moving forward      
INIT_FWD          BCLR    PORTA, %00000011        ; Clear both motor direction bits for forward motion
                  BSET    PTT, %00110000          ; Set specific bits in PTT for forward movement
                  RTS                             ; Return from the subroutine
                 
; Sets up for moving backwards  
INIT_REV          BSET    PORTA,%00000011         ; Set both motor direction bits for reverse motion
                  BSET    PTT,%00110000           ; Set specific bits in PTT for backward movement
                  RTS                             ; Return from the subroutine

; Turns off both motors                  
INIT_STOP         BCLR    PTT, %00110000          ; Clear specific bits to stop both motors
                  RTS                             ; Return from the subroutine



;***************************************************************************************************
;       Initialize Sensors
INIT              BCLR   DDRAD,$FF ; Make PORTAD an input (DDRAD @ $0272)
                  BSET   DDRA,$FF  ; Make PORTA an output (DDRA @ $0002)
                  BSET   DDRB,$FF  ; Make PORTB an output (DDRB @ $0003)
                  BSET   DDRJ,$C0  ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
                  RTS


;***************************************************************************************************
;        Initialize ADC              
openADC           MOVB   #$80,ATDCTL2    ; Turn on ADC (ATDCTL2 @ $0082)
                  LDY    #1              ; Wait for 50 us for ADC to be ready
                  JSR    del_50us        ; - " -
                  MOVB   #$20,ATDCTL3    ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                  MOVB   #$97,ATDCTL4    ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
                  RTS

;---------------------------------------------------------------------------
;                           Clear LCD Buffer
; This routine writes ’space’ characters (ascii 20) into the LCD display
; buffer in order to prepare it for the building of a new display buffer.
; This needs only to be done once at the start of the program. Thereafter the
; display routine should maintain the buffer properly.

CLR_LCD_BUF       LDX   #CLEAR_LINE
                  LDY   #TOP_LINE
                  JSR   STRCPY
             
CLB_SECOND        LDX   #CLEAR_LINE
                  LDY   #BOT_LINE
                  JSR   STRCPY
             
CLB_EXIT          RTS

; -------------------------------------------------------------------------------------------------      
;                           String Copy
; Copies a null-terminated string (including the null) from one location to
; another
; Passed: X contains starting address of null-terminated string
; Y contains first address of destination

STRCPY            PSHX            ; Protect the registers used
                  PSHY
                  PSHA
STRCPY_LOOP       LDAA 0,X        ; Get a source character
                  STAA 0,Y        ; Copy it to the destination
                  BEQ STRCPY_EXIT ; If it was the null, then exit
                  INX             ; Else increment the pointers
                  INY
                  BRA STRCPY_LOOP ; and do it again
STRCPY_EXIT       PULA            ; Restore the registers
                  PULY
                  PULX
                  RTS  
                     
; -------------------------------------------------------------------------------------------------      
;                          Guider LEDs ON                                                
; This routine enables the guider LEDs so that readings of the sensor                              |
; correspond to the ’illuminated’ situation.                                                       |
; Passed: Nothing                                                                                  |
; Returns: Nothing                                                                                 |
; Side: PORTA bit 5 is changed                                                                     |
; -------------------------------------------------------------------------------------------------      
G_LEDS_ON         BSET PORTA,%00100000 ; Set bit 5                                                 |
                  RTS                                                                             ;|
; -------------------------------------------------------------------------------------------------      


; -------------------------------------------------------------------------------------------------      
;                          Guider LEDs OFF                                          
; This routine disables the guider LEDs. Readings of the sensor                                    |
; correspond to the ’ambient lighting’ situation.                                                  |
; Passed: Nothing                                                                                  |
; Returns: Nothing                                                                                 |
; Side: PORTA bit 5 is changed                                                                     |
; -------------------------------------------------------------------------------------------------                                                                                                      
G_LEDS_OFF        BCLR PORTA,%00100000 ; Clear bit 5                                               |
                  RTS                                                                             ;|
; -------------------------------------------------------------------------------------------------      

             
; -------------------------------------------------------------------------------------------------      
;                           Read Sensors
;
; This routine reads the eebot guider sensors and puts the results in RAM
; registers.
 
; Note: Do not confuse the analog multiplexer on the Guider board with the
; multiplexer in the HCS12.The guider board mux must be set to the
; appropriate channel using the SELECT_SENSOR routine. The HCS12 always
; reads the selected sensor on the HCS12 A/D channel AN1.

; The A/D conversion mode used in this routine is to read the A/D channel
; AN1 four times into HCS12 data registers ATDDR0,1,2,3. The only result
; used in this routine is the value from AN1, read from ATDDR0. However,
; other routines may wish to use the results in ATDDR1, 2 and 3.
; Consequently, Scan=0, Mult=0 and Channel=001 for the ATDCTL5 control word.
; Passed:   None
; Returns:  Sensor readings in:
;           SENSOR_LINE(0)(SensorE/F)
;           SENSOR_BOW (1)(SensorA)
;           SENSOR_PORT(2)(SensorB)
;           SENSOR_MID (3)(SensorC)
;           SENSOR_STBD(4)(SensorD)
; Note:
;       The sensor number is shown in brackets
;
; Algorithm:
;       Initialize the sensor number to 0
;       Initialize a pointer in to the RAM at the start of the Sensor Array storage
; Loop  Store %10000001 to the ATDCTL5(to select AN1 and start a conversion)
;       Repeat
;         Read ATDSTAT0
;       Until Bit SCF of ATDSTAT0 == 1 (at which time the conversion is complete)
;       Store the contents of ATDDR0L at the pointer
;       If the pointer is at the last entry in Sensor Array, then
;         Exit
;       Else
;         Increment the sensor number
;         Increment the pointer
;       Loop again.
; -------------------------------------------------------------------------------------------------      
READ_SENSORS      CLR   SENSOR_NUM        ; Select sensor number 0
                  LDX   #SENSOR_LINE      ; Point at the start of the sensor array
RS_MAIN_LOOP      LDAA  SENSOR_NUM        ; Select the correct sensor input
                  JSR   SELECT_SENSOR     ; on the hardware
                  LDY   #400              ; 20 ms delay to allow the
                  JSR   del_50us          ; sensor to stabilize
                  LDAA  #%10000001        ; Start A/D conversion on AN1
                  STAA  ATDCTL5
                  BRCLR ATDSTAT0,$80,*    ; Repeat until A/D signals done
                  LDAA  ATDDR0L           ; A/D conversion is complete in ATDDR0L
                  STAA  0,X               ; so copy it to the sensor register
                  CPX   #SENSOR_STBD      ; If this is the last reading
                  BEQ   RS_EXIT           ; Then exit
                  INC   SENSOR_NUM        ; Else, increment the sensor number
                  INX                     ; and the pointer into the sensor array
                  BRA   RS_MAIN_LOOP      ; and do it again
RS_EXIT           RTS
             

; -------------------------------------------------------------------------------------------------      
;                               Select Sensor

;Thisroutineselectsthesensornumberpassed inACCA. Themotordirection 
; bits 0,1, the guidersensorselectbit 5 and the unusedbits6,7inthe 
; same machine registerPORTAarenotaffected. 
;BitsPA2,PA3,PA4areconnectedtoa74HC4051analogmuxontheguiderboard, 
; which selectstheguider sensortobeconnectedto AN1. 
;Passed: SensorNumberinACCA 
;Returns:Nothing 
;SideEffects:ACCAischanged 
;Algorithm: 
;First, copythecontentsofPORTAintoatemporary locationTEMPandclear 
; thesensorbits2,3,4intheTEMPtozeros by ANDingitwiththemask 
; 11100011.Thezerosinthemaskclear the correspondingbitsinthe 
; TEMP.The1’shavenoeffect. 
;Next,move thesensorselectionnumberlefttwopositions toalignit 
; withthecorrectbitpositionsfor sensor selection. 
;Clearallthebitsaroundthe(shifted) sensor number byANDingitwith 
; themask 00011100. The zerosinthemaskclear everythingexcept 
; thesensornumber. 
;Now we can combinethesensornumberwith the TEMP usinglogicalOR. 
; Theeffect isthat onlybits2,3,4arechanged in the TEMP,andthese 
; bits now correspondto the sensornumber. 
;Finally,savetheTEMPtothehardware.
; -------------------------------------------------------------------------------------------------      
SELECT_SENSOR     PSHA              ; Save the sensor number for the moment
                  LDAA PORTA        ; Clear the sensor selection bits to zeros
                  ANDA #%11100011   ;
                  STAA TEMP         ; and save it into TEMP
                  PULA              ; Get the sensor number
                  ASLA              ; Shift the selection number left, twice
                  ASLA ;
                  ANDA #%00011100   ; Clear irrelevant bit positions
                  ORAA TEMP         ; OR it into the sensor bit positions
                  STAA PORTA        ; Update the hardware
                  RTS

DP_FRONT_SENSOR   EQU TOP_LINE+3
DP_PORT_SENSOR    EQU BOT_LINE+0
DP_MID_SENSOR     EQU BOT_LINE+3
DP_STBD_SENSOR    EQU BOT_LINE+6
DP_LINE_SENSOR    EQU BOT_LINE+9
             
DISPLAY_SENSORS   LDAA  SENSOR_BOW        ; Get the FRONT sensor value
                  JSR   BIN2ASC           ; Convert to ascii string in D
                  LDX   #DP_FRONT_SENSOR  ; Point to the LCD buffer position
                  STD   0,X               ; and write the 2 ascii digits there
                 
                  LDAA  SENSOR_PORT       ; Repeat for the PORT value
                  JSR   BIN2ASC
                  LDX   #DP_PORT_SENSOR
                  STD   0,X
                 
                  LDAA  SENSOR_MID        ; Repeat for the MID value
                  JSR   BIN2ASC
                  LDX   #DP_MID_SENSOR
                  STD   0,X
                 
                  LDAA  SENSOR_STBD       ; Repeat for the STARBOARD value
                  JSR   BIN2ASC
                  LDX   #DP_STBD_SENSOR
                  STD   0,X
                 
                  LDAA  SENSOR_LINE       ; Repeat for the LINE value
                  JSR   BIN2ASC
                  LDX   #DP_LINE_SENSOR
                  STD   0,X
                 
                  LDAA  #CLEAR_HOME       ; Clear the display and home the cursor
                  JSR   cmd2LCD           ; "
                 
                  LDY   #40               ; Wait 2 ms until "clear display" command is complete
                  JSR   del_50us
                 
                  LDX   #TOP_LINE         ; Now copy the buffer top line to the LCD
                  JSR   putsLCD
                 
                  LDAA  #LCD_SEC_LINE     ; Position the LCD cursor on the second line
                  JSR   LCD_POS_CRSR
                 
                  LDX   #BOT_LINE         ; Copy the buffer bottom line to the LCD
                  JSR   putsLCD
                  RTS
                  
;***************************************************************************************************
; Binary to ASCII 
; Converts an8bitbinary valueinACCAtotheequivalent ASCIIcharacter2 
; character stringinaccumulatorD 
; Uses atable-drivenmethod ratherthanvarioustricks. 
; Passed:Binary value inACCA 
; Returns: ASCII Character stringinD 
; Side Fx: ACCBis destroyed

HEX_TABLE             FCC "0123456789ABCDEF"  ; Table for converting values

BIN2ASC               PSHA                    ; Save a copy of the input number
                      TAB            
                      ANDB #%00001111         ; Strip off the upper nibble
                      CLRA                    ; D now contains 000n where n is the LSnibble
                      ADDD #HEX_TABLE         ; Set up for indexed load
                      XGDX                
                      LDAA 0,X                ; Get the LSnibble character

                      PULB                    ; Retrieve the input number into ACCB
                      PSHA                    ; and push the LSnibble character in its place
                      RORB                    ; Move the upper nibble of the input number
                      RORB                    ;  into the lower nibble position.
                      RORB
                      RORB
                      ANDB #%00001111         ; Strip off the upper nibble
                      CLRA                    ; D now contains 000n where n is the MSnibble
                      ADDD #HEX_TABLE         ; Set up for indexed load
                      XGDX                                                              
                      LDAA 0,X                ; Get the MSnibble character into ACCA
                      PULB                    ; Retrieve the LSnibble character into ACCB

                      RTS

;--------------------------------------------------------------------------
; Routines to control the Liquid Crystal Display
;--------------------------------------------------------------------------

; Initialize the LCD
openLCD LDY #2000         ; Wait 100ms for LCD to be ready
    JSR del_50us      ; " 
    LDAA #INTERFACE   ; Set 8-bit data, 2-line display, 5x8 font
    JSR cmd2LCD       ; " 
    LDAA #CURSOR_OFF  ; Display on, cursor off, blinking off
    JSR cmd2LCD       ; " 
    LDAA #SHIFT_OFF   ; Move cursor right (address increments, no char. shift)
    JSR cmd2LCD       ; " 
    LDAA #CLEAR_HOME  ; Clear the display and home the cursor
    JSR cmd2LCD       ; " 
    LDY #40           ; Wait 2 ms until "clear display" command is complete
    JSR del_50us      ; " 
    RTS
    
;***************************************************************************************************
clrLCD:           LDAA  #$01
                  JSR   cmd2LCD
                  LDY   #40
                  JSR   del_50us
                  RTS
    
;--------------------------------------------------------------------------

; Send a command in accumulator A to the LCD
cmd2LCD BCLR LCD_CNTR, LCD_RS  ; Select the LCD Instruction register
    JSR dataMov           ; Send data to IR or DR of the LCD
    RTS

;--------------------------------------------------------------------------

; Send a character in accumulator A to LCD
putcLCD BSET LCD_CNTR, LCD_RS  ; Select the LCD Data register
    JSR dataMov           ; Send data to IR or DR of the LCD
    RTS

;--------------------------------------------------------------------------

; Send a NULL-terminated string pointed to by X
putsLCD LDAA 1, X+         ; Get one character from the string
    BEQ donePS         ; Reach NULL character?
    JSR putcLCD
    BRA putsLCD
donePS RTS

;--------------------------------------------------------------------------

; Send data to the LCD IR or DR depending on the RS signal
dataMov BSET LCD_CNTR, LCD_E    ; Pull the LCD E-signal high
    STAA LCD_DAT           ; Send the 8 bits of data to LCD
    NOP
    NOP
    NOP
    BCLR LCD_CNTR, LCD_E    ; Pull the E signal low to complete the write operation
    LDY #1                 ; Adding this delay will complete the internal
    JSR del_50us           ; Operation for most instructions
    RTS

;--------------------------------------------------------------------------

; Position the Cursor
; This routine positions the display cursor in preparation for the writing
; of a character or string.
; For a 20x2 display:
; The first line of the display runs from 0..19.
; The second line runs from 64..83.
; The control instruction to position the cursor has the format:
; 1aaaaaaa
; where aaaaaa is a 7-bit address.
; Passed: 7-bit cursor Address in ACCA
; Returns: Nothing
; Side Effects: None
LCD_POS_CRSR ORAA #%10000000    ; Set the high bit of the control word
    JSR cmd2LCD        ; and set the cursor address
    RTS

;--------------------------------------------------------------------------

; 50 Microsecond Delay
del_50us PSHX               ; (2 E-clk) Protect the X register
eloop LDX #300           ; (2 E-clk) Initialize the inner loop counter
iloop NOP                ; (1 E-clk) No operation
    DBNE X, iloop      ; (3 E-clk) If the inner counter not 0, loop again
    DBNE Y, eloop      ; (3 E-clk) If the outer counter not 0, loop again
    PULX               ; (3 E-clk) Restore the X register
    RTS                ; (5 E-clk) Else return

;--------------------------------------------------------------------------
initAD            MOVB  #$C0,ATDCTL2    ;power up AD, select fast flag clear
                  JSR   del_50us        ;wait for 50 us
                  MOVB  #$00,ATDCTL3    ;8 conversions in a sequence
                  MOVB  #$85,ATDCTL4    ;res=8, conv-clks=2, prescal=12
                  BSET  ATDDIEN,$0C     ;configure pins AN03,AN02 as digital inputs
                  RTS  
 ;***************************************************************************************************
int2BCD           XGDX              ;Save the binary number into .X
                  LDAA #0           ;Clear the BCD_BUFFER
                  STAA TEN_THOUS
                  STAA THOUSANDS
                  STAA HUNDREDS
                  STAA TENS
                  STAA UNITS
                  STAA BCD_SPARE
                  STAA BCD_SPARE+1
                 
                  CPX #0            ; Check for a zero input
                  BEQ CON_EXIT      ; and if so, exit
                 
                  XGDX              ; Not zero, get the binary number back to .D as dividend
                  LDX #10           ; Setup 10 (Decimal!) as the divisor
                  IDIV              ; Divide Quotient is now in .X, remainder in .D
                  STAB UNITS        ; Store remainder
                  CPX #0            ; If quotient is zero,
                  BEQ CON_EXIT      ; then exit
                 
                  XGDX              ; else swap first quotient back into .D
                  LDX #10           ; and setup for another divide by 10
                  IDIV
                  STAB TENS
                  CPX #0
                  BEQ CON_EXIT
                 
                  XGDX              ; Swap quotient back into .D
                  LDX #10           ; and setup for another divide by 10
                 
                  IDIV
                  STAB HUNDREDS
                  CPX #0
                  BEQ CON_EXIT
                 
                  XGDX              ; Swap quotient back into .D
                  LDX #10           ; and setup for another divide by 10
                  IDIV
                  STAB THOUSANDS
                  CPX #0
                  BEQ CON_EXIT
                 
                  XGDX              ; Swap quotient back into .D
                  LDX #10           ; and setup for another divide by 10
                  IDIV
                  STAB TEN_THOUS
           
CON_EXIT          RTS               ; We’re done the conversion              

;***************************************************************************************************
;* BCD to ASCII Conversion Routine
;* This routine converts the BCD number in the BCD_BUFFER
;* into ascii format, with leading zero suppression.
;* Leading zeros are converted into space characters.
;* The flag ’NO_BLANK’ starts cleared and is set once a non-zero
;* digit has been detected.
;* The ’units’ digit is never blanked, even if it and all the
;* preceding digits are zero.

BCD2ASC           LDAA    #0            ; Initialize the blanking flag
                  STAA    NO_BLANK
             
C_TTHOU           LDAA    TEN_THOUS     ; Check the ’ten_thousands’ digit
                  ORAA    NO_BLANK
                  BNE     NOT_BLANK1

ISBLANK1          LDAA    #' '          ; It’s blank
                  STAA    TEN_THOUS     ; so store a space
                  BRA     C_THOU        ; and check the ’thousands’ digit
             
NOT_BLANK1        LDAA    TEN_THOUS     ; Get the ’ten_thousands’ digit
                  ORAA    #$30          ; Convert to ascii
                  STAA    TEN_THOUS
                  LDAA    #$1           ; Signal that we have seen a ’non-blank’ digit
                  STAA    NO_BLANK

C_THOU            LDAA    THOUSANDS     ; Check the thousands digit for blankness
                  ORAA    NO_BLANK      ; If it’s blank and ’no-blank’ is still zero
                  BNE     NOT_BLANK2

ISBLANK2          LDAA    #' '          ; Thousands digit is blank
                  STAA    THOUSANDS     ; so store a space
                  BRA     C_HUNS        ; and check the hundreds digit

NOT_BLANK2        LDAA    THOUSANDS     ; (similar to ’ten_thousands’ case)
                  ORAA    #$30
                  STAA    THOUSANDS
                  LDAA    #$1
               
                  STAA    NO_BLANK
             
C_HUNS            LDAA    HUNDREDS      ; Check the hundreds digit for blankness
                  ORAA    NO_BLANK      ; If it’s blank and ’no-blank’ is still zero
                  BNE     NOT_BLANK3

ISBLANK3          LDAA    #' '          ; Hundreds digit is blank
                  STAA    HUNDREDS       ; so store a space
                  BRA     C_TENS          ; and check the tens digit

NOT_BLANK3        LDAA    HUNDREDS          ; (similar to ’ten_thousands’ case)
                  ORAA    #$30
                  STAA    HUNDREDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_TENS            LDAA    TENS          ; Check the tens digit for blankness
                  ORAA    NO_BLANK      ; If it’s blank and ’no-blank’ is still zero
                  BNE     NOT_BLANK4

ISBLANK4          LDAA    #' '          ; Tens digit is blank
                  STAA    TENS          ; so store a space
                  BRA     C_UNITS       ; and check the units digit

NOT_BLANK4        LDAA    TENS          ; (similar to ’ten_thousands’ case)
                  ORAA    #$30
                  STAA    TENS

C_UNITS           LDAA    UNITS         ; No blank check necessary, convert to ascii.
                  ORAA    #$30
                  STAA    UNITS

                  RTS                 ; We’re done

;***************************************************************************************************
ENABLE_TOF        LDAA    #%10000000
                  STAA    TSCR1       ; Enable TCNT
                  STAA    TFLG2       ; Clear TOF
                  LDAA    #%10000100  ; Enable TOI and select prescale factor equal to 16
                  STAA    TSCR2
                  RTS
;***************************************************************************************************
TOF_ISR           INC     TOF_COUNTER
                  LDAA    #%10000000  ; Clear
                  STAA    TFLG2       ; TOF
                  RTI
;***************************************************************************************************
;*  Update Display (Battery Voltage + Current State)        *
;***************************************************************************************************
UPDT_DISPL      LDAA  #$82                      ;
                JSR   cmd2LCD                   ;
               
                LDAB  CRNT_STATE                ; 
                LSLB                            ; 
                LSLB                            ; 
                LSLB                            ; 
                LDX   #tab                      ; 
                ABX                             ; 
                JSR   putsLCD                   ; 
           
                LDAA  #$8F                      ;
                JSR   cmd2LCD                   ;
                LDAA  SENSOR_BOW                ;
                JSR   BIN2ASC                   ;
                JSR   putcLCD                   ;
                EXG   A,B                       ;
                JSR   putcLCD                   ;

                LDAA  #$92                      ;
                JSR   cmd2LCD                   ; 
                LDAA  SENSOR_LINE               ; 
                JSR   BIN2ASC                   ; 
                JSR   putcLCD                   ;
                EXG   A,B                       ;
                JSR   putcLCD                   ;

                LDAA  #$CC                      ; Move LCD cursor to Port position on 2nd row
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_PORT               ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""

                LDAA  #$CF                      ; Move LCD cursor to Mid position on 2nd row
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_MID                ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""

                LDAA  #$D2                      ; Move LCD cursor to Starboard position on 2nd row
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_STBD               ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""
       
                MOVB  #$90,ATDCTL5              ; R-just., uns., sing. conv., mult., ch=0, start
                BRCLR ATDSTAT0,$80,*            ; Wait until the conver. seq. is complete
                LDAA  ATDDR0L                   ; Load the ch0 result - battery volt - into A
                LDAB  #39                       ; AccB = 39
                MUL                             ; AccD = 1st result x 39
                ADDD  #600                      ; AccD = 1st result x 39 + 600
                JSR   int2BCD
                JSR   BCD2ASC
                LDAA  #$C2                      ; move LCD cursor to the end of msg3
                JSR   cmd2LCD                   ; "                
                LDAA  TEN_THOUS                 ; output the TEN_THOUS ASCII character
                JSR   putcLCD                   ; "
                LDAA  THOUSANDS                 ; output the THOUSANDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  #$2E                      ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  HUNDREDS                  ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "                

                LDAA  #$C9                      ; Move LCD cursor to the end of msg4
                JSR   cmd2LCD
               
                BRCLR PORTAD0,#%00000100,bowON  ; If FWD_BUMP, then
                LDAA  #$20                      ;
                JSR   putcLCD                   ;
                BRA   stern_bump                ; Display 'B' on LCD
bowON:          LDAA  #$42                      ; ""
                JSR   putcLCD                   ; ""
         
stern_bump:     BRCLR PORTAD0,#%00001000,sternON; If REV_BUMP, then
                LDAA  #$20                      ;
                JSR   putcLCD                   ;
                BRA   UPDT_DISPL_EXIT           ; Display 'S' on LCD
sternON:        LDAA  #$53                      ; ""
                JSR   putcLCD                   ; ""
                
UPDT_DISPL_EXIT RTS                             ; and exit

;***************************************************************************************************
;*                                Interrupt Vectors                                                *
;***************************************************************************************************
                  ORG     $FFFE
                  DC.W    Entry ; Reset Vector
                  ORG     $FFDE
                  DC.W    TOF_ISR ; Timer Overflow Interrupt Vector