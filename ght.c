/*
Ez egy RaspberryPi-ra (látszik is az inlclude-ban, 20 pinje ban a raspberrynek és azokkal kellet irányítani egy I2C-t) 
írt  teszt program, amit ki kellett javítanom/egészítenek. 
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringShift.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <stdint.h>
#include <time.h>
 
// change this to 2 for quick testing, keep it 18 otherwise
#define ELECTRODES 2
 
// time constants in seconds
#define SHORT_MODE_WAIT             70
#define TRIANGLE_MODE_WAIT          4
#define SWEEP_MODE_WAIT             10
#define CMRR_WAIT                   70
#define TRANSIENTS_WAIT             5
#define BATEMU_WAIT                 0.1
#define SYNC_WAIT                   0.5
#define SINE_TIME                   3 
 
// you can use this for triangle/sweep mode waits:
// delayMicroseconds(4e6*(1/6.0)); // wait four periods
 
// the number of frequencies in sweep mode
#define FREQS   7
 
// F1, F2 (not used)
// used to generate logscale frequencies from F1 TO F2
#define F1      0.5
#define F2      50
 
// CMRR
#define CMRR_FREQ 50
 
// pins
#define TI_595_SRCK         10
#define TI_595_RCK          14
#define TI_595_SER_OUT      13
#define TI_595_SER_IN       12
#define AD7994_CONVST       5
 
#define TI_595_DDS_FSELECT  0
#define TI_595_DDS_RESET    1
#define TI_595_DDS_SDATA    2
#define TI_595_DDS_FSYNC    3
#define TI_595_DDS_SCLK     5
#define TI_595_DDS_SLEEP    6
#define TI_595_DDS_PSELECT  7
 
#define TI_595_AC_DAC_CLK   4
#define TI_595_AC_DAC_SRI   8
#define TI_595_AC_DAC_LD    15
 
// Kocka
#define TI_595_KOCKA_K4     67 
#define TI_595_KOCKA_K3     66
#define TI_595_KOCKA_K2     61
#define TI_595_KOCKA_SDI    12
#define TI_595_KOCKA_SYNC   13
#define TI_595_KOCKA_SCLK   14
 
// sync pins
#define PI_IR_REC           0
#define PI_PHTRIG           1
 
///structures
 
// battery temperature selector
typedef enum 
{ 
    BATTERY_TEMP_COLD,
    BATTERY_TEMP_COOL,
    BATTERY_TEMP_NORMAL,
    BATTERY_TEMP_WARM,
    BATTERY_TEMP_HOT
} battery_temperature;
 
// enable, disable enum
typedef enum 
{
    DISABLE,
    ENABLE
} enable_t;
 
// waveform selector
typedef enum 
{
    SINE,
    TRIANGLE
} wave_t;
 
// determines if CZ relay is to be skipped or not
typedef enum
{
    INCLUDING_CZ,
    EXCLUDING_CZ
} skip_cz_t;
 
// Global variables
// this variable holds the status of the 595 chips
unsigned char TI_595_state_vector[80]; 
 
// file handles for I2C devices
int file_handle_ADC, file_handle_DAC, file_handle_DigPot; 
 
// this should not be a global variable
int error_code; 
 
// used to iterate through the electrode relays
// this variable is modified by increment_relay_address(char skip_cz)
// and also modified by reset_relay_address()
int relay_address;
 
 
// !!! T5 and T6 SPST/SPDT re in reverse order
void increment_relay_address(skip_cz_t skip_cz)
{
    // increment relay_address by 2
    relay_address = relay_address + 2;
    
    // skip CZ if neccesary
    if (relay_address == 35 && skip_cz == EXCLUDING_CZ)
    {
        relay_address = relay_address + 2;
    }
    
    // skip to T5
    if (relay_address == 57)
    {
        relay_address = 75;
    }   
}
 
// resets electrode address to FP1
void reset_relay_address()
{
    relay_address = 23;
}
 
void USB_5v_out_enable(enable_t e)
{
    if(e == ENABLE)
    {
        printf("Enabling 5V USB out...");
        TI_595_state_vector[57] = '1';
    }
    else
    {
        printf("Disabling 5V USB out...");
        TI_595_state_vector[57] = '0';
    }
    printf(" - OK\n");
}
 
void init_i2c_devices()
{
    printf("Initializing digital potentiometer...");
    file_handle_DigPot = wiringPiI2CSetup (0x18);
    if (file_handle_DigPot == -1) 
    {
        printf(" - UNREACHABLE\n");
    }
    else
    {
        printf(" - OK\n");
    }
    
    printf("Initializing AD5696R DAC...");
    file_handle_DAC = wiringPiI2CSetup (0x0C);
    if (file_handle_DAC == -1) 
    {
        printf(" - UNREACHABLE\n"); 
    }
    else
    {
        printf(" - OK\n");
    }
    
    printf("Initializing AD7994-1 ADC...");
    file_handle_ADC = wiringPiI2CSetup (0x23);
    if (file_handle_ADC == -1) 
    {
        printf(" - UNREACHABLE\n"); 
    }
    else
    {
        printf(" - OK\n");
    }
}
 
void update_595_state(void)
{
    unsigned int i;
    digitalWrite(TI_595_SER_OUT, LOW);
    digitalWrite(TI_595_SRCK, LOW);
    
    for (i=0; i<80; i++)
    {
        if (TI_595_state_vector[79-i] == '0') 
        {
            digitalWrite(TI_595_SER_OUT, LOW);
        }
        else 
        {
            digitalWrite(TI_595_SER_OUT, HIGH);
        }
        delayMicroseconds(1);
        digitalWrite(TI_595_SRCK, HIGH);
        delayMicroseconds(1);
        digitalWrite(TI_595_SRCK, LOW);
    }
    digitalWrite(TI_595_SER_OUT, LOW);
    delayMicroseconds(1);
    digitalWrite(TI_595_RCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(TI_595_RCK, LOW);
    delayMicroseconds(1);   
}
 
// clocks out 16 bits of data
// NO FSYNC!
// used in DDS functions
void dds_send(uint16_t data)
{
    char bit;
    
    for(int i=15; i>=0 ; i--)
    {
        // put sdata out
        bit = (data >> i) & 0x0001;
        if (bit == 0)
        {
            TI_595_state_vector[TI_595_DDS_SDATA] = '1';
        }
        else 
        {
            TI_595_state_vector[TI_595_DDS_SDATA] = '0';
        }
        update_595_state();
        
        // clock sdata
        TI_595_state_vector[TI_595_DDS_SCLK] = '1';
        update_595_state();
        TI_595_state_vector[TI_595_DDS_SCLK] = '0';
        update_595_state(); 
    }   
}
 
// intilaizes DDS
// input: SINE or TRIANGLE
void dds_init(wave_t waveform)
{
    printf("Initializing DDS...");
    
    // set 595 pins to initial values
    TI_595_state_vector[TI_595_DDS_FSYNC]   = '0';
    TI_595_state_vector[TI_595_DDS_SCLK]    = '0';
    TI_595_state_vector[TI_595_DDS_SDATA]   = '1';
    TI_595_state_vector[TI_595_DDS_FSELECT] = '1';
    TI_595_state_vector[TI_595_DDS_PSELECT] = '1';
    TI_595_state_vector[TI_595_DDS_SLEEP]   = '1';
    TI_595_state_vector[TI_595_DDS_RESET]   = '1';
    update_595_state();
    
    
    /* set the configuration register
    DB15 - 0
    DB14 - 0
    DB13 - 1: 28 bit mode enable
    DB12 - 0: half resolution mode, ingored since DB13 is 1
    DB11 - 0: FSEL - ignored: pin mode activated
    DB10 - 0: PSEL - ignored: pin mode activated
    DB09 - 1: FSEL, PSEL, SLEEP, RESET controlled by pins
    DB08 - 0: RESET, ignored
    DB07 - 0: SLEEP1, not reset, ignored
    DB06 - 0: SLEEP12, DAC enabled
    DB05 - 0: OPBITEN, sign bit not needed
    DB04 - 0: not needed
    DB03 - 0: not needed
    DB02 - 0: must be set to 0
    DB01 - 0: sign bit mode, not needed
    DB00 - 0: must be set to 0
    
    REGISTER VALUE: 0b0010001000000000
    */
 
    TI_595_state_vector[TI_595_DDS_FSYNC] = '1';
    update_595_state();
    
    if (waveform == TRIANGLE)
    {
        dds_send(0b0010001000000010);
    }
    else
    {
        dds_send(0b0010001000000000);
    }
    
    TI_595_state_vector[TI_595_DDS_RESET]   = '0';
    update_595_state();
    
    TI_595_state_vector[TI_595_DDS_RESET]   = '1';
    update_595_state();
        
    TI_595_state_vector[TI_595_DDS_FSYNC] = '1';
    update_595_state();
    
    printf(" - OK\n");
}
 
void dds_set_frequency(double f)
{
    const double fmck = 6000000;
    uint32_t freg = 0;
    uint16_t freg_lsb = 0;
    uint16_t freg_msb = 0;
    static char dds_fsel = 0;
    
    printf("Setting DDS frequency to %f Hz\n", f);
    
    freg = f*(pow(2, 28)/fmck);
    
    freg_lsb = freg & 0x3FFF;
    freg_msb = (freg >> 14) & 0x3FFF;
    
    // swaps between FTWs when changing frequency
    if (dds_fsel == 0)
    {
        dds_fsel = 1;
        freg_lsb = freg_lsb | 0x8000;
        freg_msb = freg_msb | 0x8000;
    }
    else
    {
        dds_fsel = 0;
        freg_lsb = freg_lsb | 0x4000;
        freg_msb = freg_msb | 0x4000;
    }
    
    TI_595_state_vector[TI_595_DDS_FSYNC] = '1';
    update_595_state();
    dds_send(freg_lsb);
    dds_send(freg_msb);
    TI_595_state_vector[TI_595_DDS_FSYNC] = '0';
    update_595_state();
        
    if (dds_fsel == 0)
    {
        TI_595_state_vector[TI_595_DDS_FSELECT] = '1';
    }
    else
    {
        TI_595_state_vector[TI_595_DDS_FSELECT] = '0';
    }
    update_595_state();
}
 
// m is for multiplication factor [0, 1]
void ac_dac_set(double m)
{
    char bit;
    uint16_t reg = 0;
    
    printf("Setting AC_DAC to %3.1f\%\n", m*100);
    
    // coerce m to [0, 1]
    if (m > 1) m = 1;
    if (m < 0) m = 0;
    
    reg = m*(pow(2, 12)-1);
    reg = reg << 4;
    reg = reg >> 4;
    TI_595_state_vector[TI_595_AC_DAC_LD] = '1';
    update_595_state();
    TI_595_state_vector[TI_595_AC_DAC_LD] = '0';
    update_595_state();
    
    for(int i=11; i>=0 ; i--)
    {
        TI_595_state_vector[TI_595_AC_DAC_CLK] = '1';
        update_595_state();
        
        // put sdata out
        bit = (reg >> i) & 0x0001;
        if (bit == 0)
        {
            TI_595_state_vector[TI_595_AC_DAC_SRI] = '1';
        }
        else 
        {
            TI_595_state_vector[TI_595_AC_DAC_SRI] = '0';
        }
        update_595_state();
        TI_595_state_vector[TI_595_AC_DAC_CLK] = '0';
        update_595_state(); 
    }
    
    TI_595_state_vector[TI_595_AC_DAC_LD] = '1';
    update_595_state();
    TI_595_state_vector[TI_595_AC_DAC_LD] = '0';
    update_595_state();
}
 
void ac_dac_init() // initializing digit. to analog
{
    printf("Initializing AC_DAC...");
 
    TI_595_state_vector[TI_595_AC_DAC_LD] = '0';
    TI_595_state_vector[TI_595_AC_DAC_CLK] = '0';
    TI_595_state_vector[TI_595_AC_DAC_SRI] = '1';
    update_595_state();
    ac_dac_set(0);
    
    printf(" - OK\n");
}
 
/// PART: KOCKA ///
 
// drives or releases K4 on kocka
void kocka_k4(char b)
{
    if (b==1) // close
    {
        TI_595_state_vector[TI_595_KOCKA_K4] = '1';
    }
    else // open
    {
        TI_595_state_vector[TI_595_KOCKA_K4] = '0';
    }
    update_595_state();
}
 
// drives or releases K3 on kocka
void kocka_k3(char b)
{
    if (b==1) // close state
    {
        TI_595_state_vector[TI_595_KOCKA_K3] = '1';
    }
    else // open state
    {
        TI_595_state_vector[TI_595_KOCKA_K3] = '0';
    }
    update_595_state();
}
 
// drives or releases K2 on kocka
void kocka_k2(char b)
{
    if (b==1) // close state
    {
        TI_595_state_vector[TI_595_KOCKA_K2] = '1';
    }
    else // open state                                                             
    {
        TI_595_state_vector[TI_595_KOCKA_K2] = '0';
    }
    update_595_state();
}
 
// sets kocka dc voltage
void kocka_set_voltage(double voltage)
{
    uint32_t command = 0;
    uint32_t dac_num = 0;
    char bit;
    
    // voltage must not exceed 0.3
    if (voltage > 0.3)
    {
        voltage = 0.3;
    }
    
    // calculate dac number
    dac_num = round((voltage/2.5)*65535);
    dac_num = dac_num << 4;
    
    // build command 
    command = 0x03;
    command = command << 20;
    command = command | dac_num;
            
    // pull up sclk
    TI_595_state_vector[TI_595_KOCKA_SCLK] = '0';
    update_595_state(); 
    
    // pull down sync
    TI_595_state_vector[TI_595_KOCKA_SYNC] = '1';
    update_595_state(); 
    
    // clock out data
    for(int i=23; i>=0 ; i--)
    {
        // put sdata out
        bit = (command >> i) & 0x0001;
        if (bit == 0)
        {
            TI_595_state_vector[TI_595_KOCKA_SDI] = '1';
        }
        else 
        {
            TI_595_state_vector[TI_595_KOCKA_SDI] = '0';
        }
        update_595_state();
        
        // clock sdata
        TI_595_state_vector[TI_595_KOCKA_SCLK] = '1';
        update_595_state();
        TI_595_state_vector[TI_595_KOCKA_SCLK] = '0';
        update_595_state(); 
    }
    
    // pull up sync
    TI_595_state_vector[TI_595_KOCKA_SYNC] = '0';
    update_595_state(); 
}
 
// Initializes kocka relays and the kocka DAC
void kocka_init()
{
    printf("Initializing KOCKA...");
    
    // pull up sclk
    TI_595_state_vector[TI_595_KOCKA_SCLK] = '0';
    
    // pull up sync
    TI_595_state_vector[TI_595_KOCKA_SYNC] = '0';
    update_595_state(); 
    
    // release all relays
    kocka_k2(0);
    kocka_k3(0);
    
    // enable kocka VDD
    kocka_k4(1); 
    kocka_set_voltage(0.0);
    printf(" - OK\n");
}
 
// applies v Volts DC through kocka
void kocka(double v)
{
    printf("Setting kocka voltage to %f V...", v);
    
    kocka_set_voltage(0.0); // first set kocka voltage to 0.0 V
    
    // release battery (shorts kocka)
    kocka_k2(0);
    kocka_k3(0);
    
    if (v == 0)
    {
        // battery already set to 0 and released
    }
    else if (v > 0)
    {
        kocka_k2(1);
        kocka_set_voltage(v);
    }
    else if (v < 0)
    {
        kocka_k3(1);
        v = fabs(v);     
    }
    
    printf(" - OK\n");
}
 
void system_init(void)
{
    printf("INIT START\n");
 
    printf("Initializing WiringPi...");
    wiringPiSetup();
    printf(" - OK\n");
    
    init_i2c_devices();
 
    printf("Initializing TI_595 pins...");
    // GPIO pins init (data direction)
    pinMode(TI_595_SRCK, OUTPUT);
    pinMode(TI_595_RCK, OUTPUT);
    pinMode(TI_595_SER_OUT, OUTPUT);
    pinMode(TI_595_SER_IN, INPUT);
    pinMode(AD7994_CONVST,OUTPUT);
    pinMode(PI_IR_REC, OUTPUT);
    pinMode(PI_PHTRIG , OUTPUT);
    
    // GPIO pins init value
    digitalWrite(TI_595_RCK, LOW);
    digitalWrite(TI_595_SRCK, LOW);
    digitalWrite(TI_595_SER_OUT, LOW);
    digitalWrite(AD7994_CONVST, LOW);
    digitalWrite(PI_IR_REC, LOW);
    digitalWrite(PI_PHTRIG, LOW);
    printf(" - OK\n");
    
    printf("Resetting 595 state vector...");
    // status of the 595 chips are set to all zero 
    for (int i = 0; i < 80; i++)
    {
        TI_595_state_vector[i]= '0'; // clear the 595 state 
    }
    update_595_state();
    printf(" - OK\n");
    
    // enable USB (drive usb_spst)
    TI_595_state_vector[57] = '1';
    update_595_state();
    
    printf("Powering USB...");
 
    dds_init(TRIANGLE);
    ac_dac_init();
    ac_dac_set(0);
    kocka_init();
    
    printf("INIT END\n");
}
 
void batemu_set_temp(battery_temperature bt)
{
    int digipot_value;
    int error_code;
    
    printf("Setting emulated temperature to ");
    
    if (bt == BATTERY_TEMP_COLD)
    {
        printf("COLD\n");
        digipot_value = 200;
    }
    else if (bt == BATTERY_TEMP_COOL)
    {
        printf("COOL\n");
        digipot_value = 123;
    }
    else if (bt == BATTERY_TEMP_NORMAL)
    {
        printf("NORMAL\n");
        digipot_value = 51;
    }
    else if (bt == BATTERY_TEMP_WARM)
    {
        printf("WARM\n");
        digipot_value = 21;
    }
    else if (bt == BATTERY_TEMP_HOT)
    {
        printf("WARM\n");
        digipot_value = 10;
    }
    else 
    {
        printf("NORMAL\n");
        digipot_value = 51;
    }
    
    error_code = wiringPiI2CWriteReg8 (file_handle_DigPot, 0, digipot_value);
    if (error_code == -1)
    {
        printf("Failed to set emulated temperature\r\n");
    }
}
 
// reads batemu current
double batemu_get_current()
{
    double voltage;
    double current;
    uint32_t adc_readout;
    
    wiringPiI2CReadReg8(file_handle_ADC, 0x18);
    
    digitalWrite(AD7994_CONVST, HIGH); // CONVST Pulsed
    delayMicroseconds(3);
    digitalWrite(AD7994_CONVST, LOW);
    delayMicroseconds(3);
 
    adc_readout = wiringPiI2CReadReg16(file_handle_ADC, 0x00);
    adc_readout = ((adc_readout&0xFF)<<8) | (adc_readout>>8);
    printf("adc_readout: %d\n", adc_readout);
    voltage = ((adc_readout/4096.0)*5.0); // calculate ADC voltage
    //printf("voltage: %f\n", voltage);
    voltage = voltage-2.5;
    voltage = voltage/11.361; // calculate actual voltage
    current = voltage*3.0; // divide by 1/3 Ohms
    
    return -current;
}
 
void batemu_enable(enable_t e)
{
    if (e == ENABLE)
    {
        printf("Enabling battery emulation...");
        TI_595_state_vector[16] = '0';  // battery emulation enabled
        TI_595_state_vector[21] = '1';  // red LED is ON
    }
    else
    {
        printf("Disabling battery emulation...");
        TI_595_state_vector[16] = '1';  // battery emulation disabled
        TI_595_state_vector[21] = '0';  // red LED is OFF
    }
    update_595_state();
    printf(" - OK\n");
}
 
int AD5696R_set_voltage(double voltage, int channel)
{
    uint16_t dac_value;
    uint16_t tmp;
    uint8_t command;
    int error_code;
    
    if (channel == 0)
    {
        command = 0x31;
    }
    else if (channel == 1)
    {
        command = 0x32;
    }
    else 
    {
        command = 0x31;
    }
    
    // setting the output voltage 
    tmp = round((voltage/5.0)*(pow(2, 16)-1)); // (2^16)-1 = 65535
    
    dac_value = 0;
    dac_value = tmp << 8;
    dac_value = dac_value | (tmp >> 8);
        
    // writing DAC
    error_code = wiringPiI2CWriteReg16(file_handle_DAC, command, dac_value);
    
    return error_code;
}
 
void excitacion_dc_set_voltage(double voltage)
{
    int error_code;
    printf("Setting excitation DC voltage to %f V...", voltage);
    
    error_code = AD5696R_set_voltage(voltage, 1);
    
    if (error_code < 0) 
    {
        printf(" - FAILED\n"); 
    }
    else
    {
        printf(" - OK\n");
    }
}
 
void batemu_set_voltage(double voltage)
{
    int error_code;
    printf("Setting emulated battery voltage to %f V...", voltage);
    
    error_code = AD5696R_set_voltage(voltage, 0);
    
    if (error_code < 0) 
    {
        printf(" - FAILED\n"); 
    }
    else
    {
        printf(" - OK\n");
    }
}
 
// wait for s seconds
void wait(double s)
{
    int t1, t2;
    if (s > 0)
    { 
        t1=millis();
        t2=millis();
        while ((t2-t1)/1000.0 <= s)
        {
            t2=millis();
            
            // this printf can mess up the terminal emulator 
            printf("Elapsed time: %2.2f s of %2.2f s\r", (t2-t1)/1000.0, s);
            delayMicroseconds(1000);
        }
        printf("\n");
    }
}
 
///
/// MAIN FUNCTION
///
 
int main (int argc, char **argv)
{
    // amplitudes in triangle mode, 1 means 100%
    double range[4] = {0.1,0.2,0.5,1};
    
    // frequencies in sweep mode (in Hz)
    double freqs[] = {0.5, 1, 2, 5, 10, 20, 50};
    
    // not used anymore
    // used to generate frequencies for sweep mode
    double ratio;
    
    /// for testing if the rasp GPIO pins are working properly (not guaranteed)
    if (wiringPiSetup () == -1)
        return 1;
    
    digitalWrite (21,1);        /// by T. Peti
    delay(2000);
    digitalWrite (21,0);
    
    
    printf("Program started already.\n");
    system_init();
 
 
    
    // setup battery emulator
    batemu_set_temp(BATTERY_TEMP_COOL);
    printf("Waiting for digipot to settle...\n");
    wait(BATEMU_WAIT);
    batemu_enable(ENABLE);
    batemu_set_voltage(4.1);
 
    // DRL 
    TI_595_state_vector[65] = '0'; // do not drive DRL_SOURCE_SELECT
    TI_595_state_vector[63] = '1'; // drive DRL_JACK_SPST
    update_595_state();
    
    printf("Syncing...\n");
    for (int i=0; i<3;i++)
    {
        wait(SYNC_WAIT);
        digitalWrite(PI_IR_REC, HIGH);
        digitalWrite(PI_PHTRIG, HIGH);
        wait(SYNC_WAIT);
        digitalWrite(PI_IR_REC, LOW);
        digitalWrite(PI_PHTRIG, LOW);     
    }
    printf("Done.\n");    
    
/**** SHORT MODE BEGIN ****/
/**
    printf("Starting noise measurement...\n");
        
    // set relay configuration
    reset_relay_address();
    printf("Connecting all electrodes directly to common\n");
    for(int i = 0; i<19; i++)
    {
        // connect all electrodes directly to common (including CZ)
        if (relay_address < 75)
        {
            TI_595_state_vector[relay_address]='1';     // drive spst
            TI_595_state_vector[relay_address+1]='0';   // do not drive spdt
        }
        else
        {
            TI_595_state_vector[relay_address]='0';     // do not drive spdt 
            TI_595_state_vector[relay_address+1]='1';   // drive spst
        }
        increment_relay_address(INCLUDING_CZ);
    }
    update_595_state();
 
    printf("Waiting for transients to settle...\n");
    wait(TRANSIENTS_WAIT);
    printf("Waiting for noise measurement to finish...\n");
    wait(SHORT_MODE_WAIT);
    printf("Done.\n");
/**** SHORT MODE END ****/
    
/**** TRIANGLE MODE BEGIN ****/
 
/*
SEQUENCE:
1: select next elcetrode
2: 10%, 20%, 50%, 100% triangle without DC
3: +300mV DC with 100% triangle
4: -300mv DC with 100% triangle
5: stop sig. gen and short kocka
6: goto 1
*/
    
    printf("Starting triangle measurement...\n");
    dds_init(SINE); // newly added
    
    // set excitation DC
    excitacion_dc_set_voltage(2.5);
    
    // set DDS freq to 5 Hz (earlier 6), sinewave already set
    dds_set_frequency(5); 
    
 
    // drive LARGE_SIGNAL_AC_SPST
    printf("Driving LARGE_SIGNAL_AC_SPST\n");
    TI_595_state_vector[69]='1';
    update_595_state();
    
    reset_relay_address();
    for(int i = 0; i <  ELECTRODES; i++) // for all electrodes excluding cz
    {
        // connect electrodes directly to driven
        printf("Connecting current electrode directly to driven\n");
        if (relay_address < 75)
        {
            TI_595_state_vector[relay_address]='1';     // drive spst
            TI_595_state_vector[relay_address+1]='1';   // drive spdt
        }
        else
        {
            TI_595_state_vector[relay_address]='1';     // drive spdt 
            TI_595_state_vector[relay_address+1]='1';   // drive spst
        }
        update_595_state();

        ac_dac_set(0.5);

        
        printf("Waiting for transients to settle...\n");
        wait(SINE_TIME); // wait for sine to finish
        ac_dac_set(0);
        // switching to measuring with 1MOhm resistance
        
        if (relay_address < 75)
        {
            TI_595_state_vector[relay_address]='0';     // drive spst, connect 1MOhm
        }
        else
        {
            TI_595_state_vector[relay_address+1]='0';   // drive spst, connect 1MOhm
        }
        update_595_state();
        ac_dac_set(0.5);
        wait(SINE_TIME);
        ac_dac_set(0.0);
 
        /*
        kocka(0.3);     // +300 mV DC
        printf("Waiting for transients to settle...\n");
        wait(TRANSIENTS_WAIT);
        printf("Waiting for measurement to finish...\n");
        wait(TRIANGLE_MODE_WAIT); 
        
        // delay(10000); /// test only (T. Peti)
        
        kocka(-0.3);    // -300 mV DC
        printf("Waiting for transients to settle...\n");
        wait(TRANSIENTS_WAIT);       // transients to settle + 5 secs
        printf("Waiting for measurement to finish...\n");
        wait(TRIANGLE_MODE_WAIT);
        
        // delay(10000); /// test only (T. Peti)
        
        printf("Turning off signal generation\n");
        ac_dac_set(0);  // set triangle amplitude to zero
        kocka(0);       // reset (short) kocka
        */
                 
        // connect electrodes back to common
        printf("Reconnecting electrode to common\n");
        if (relay_address < 75)
        {
            TI_595_state_vector[relay_address]='1';     // drive spst
            TI_595_state_vector[relay_address+1]='0';   // do not drive spdt
        }
        else
        {
            TI_595_state_vector[relay_address]='0';     // do not drive spdt 
            TI_595_state_vector[relay_address+1]='1';   // drive spst
        }
        update_595_state();
        
        // next electrode
        if (i<17) printf("Selecting next electrode\n");
        increment_relay_address(EXCLUDING_CZ);
    }
    printf("Done.\n");
/**** TRIANGLE MODE END ****/
 
/**** SWEEP MODE BEGIN ****/
 
    printf("Starting sweep mode...\n");
    
    // generate logscale frequencies
    // not used, frequencies are already in the array freqs[FREQS]
    /*
    ratio = (double)FREQS/((double)FREQS-1);
    for (int i=0; i<FREQS; i++)
    {   
        freqs[i] = pow((((i*ratio)/(double)FREQS)*10), log10(F2-F1))+F1;
    }
    */
 
    printf("Initializing signal generation for sweep mode\n");
    ac_dac_set(0);          // (already zero)
    dds_init(SINE);         // select sine waveform for DDS
    dds_set_frequency(0);   // zero frequency
    
    reset_relay_address();
    for(int i = 0; i<ELECTRODES; i++) // for every electrode excluding CZ
    {
        // connect electrodes directly to driven
        printf("Connecting current electrode directly to driven\n");
        if (relay_address < 75)
        {
            TI_595_state_vector[relay_address]='1';     // drive spst
            TI_595_state_vector[relay_address+1]='1';   // drive spdt
        }
        else
        {
            TI_595_state_vector[relay_address]='1';     // drive spdt 
            TI_595_state_vector[relay_address+1]='1';   // drive spst
        }
        update_595_state();
        
        printf("Wait for transients to settle...\n");
        wait(TRANSIENTS_WAIT);
        
        // ampliutde may not be correct 
        ac_dac_set(0.2); // turn on signal generation
        for(int j = 0; j < FREQS; j++)
        {
            dds_set_frequency(freqs[j]);
            wait(SWEEP_MODE_WAIT);
            //delayMicroseconds((1/freqs[j])*4e6); // wait for four periods
        }
        
        // stop signal generation
        printf("Stopping signal generation\n");
        ac_dac_set(0);
        dds_set_frequency(0);
        
        // reset relays to common
        printf("Reconnecting electrode to common\n");
        if (relay_address < 75)
        {
            TI_595_state_vector[relay_address]='1';     // drive spst
            TI_595_state_vector[relay_address+1]='0';   // do not drive spdt
        }
        else
        {
            TI_595_state_vector[relay_address]='0';     // do not drive spdt 
            TI_595_state_vector[relay_address+1]='1';   // drive spst
        }
        update_595_state();
        
        // next electrode
        if (i<17) printf("Selecting next electrode\n");
        increment_relay_address(EXCLUDING_CZ);
    }
    printf("Done\n");
/**** SWEEP MODE END ****/
 
/**** CMRR MODE BEGIN ****/
/*
SEQUENCE
0: all elercrodes already shorted to common
1: connect next electrode to driven through 1 MOhms
2: wait for cmrr measurement
3: set +300 mV DC
4: wait for cmrr measurement
5: set -300 mV DC
6: wait for cmrr measurement
7: short kocka
8: goto 1 
*/
 
    printf("Starting CMRR measurement...\n");
    
    printf("Initializing signal generation for CMRR measurement\n");
    dds_set_frequency(CMRR_FREQ);     // set dds frequency
    ac_dac_set(1);                   // set sine amplitude
    
    printf("Driving HV_COUPLING_SPST\n");
    TI_595_state_vector[71]='1';        // drive HV_COUPLING_SPST
    
    printf("Driving COMMON_VS_MIX\n");
    TI_595_state_vector[70]='1';        // drive common_vs_mix
    
    reset_relay_address();
    for(int i = 0; i<ELECTRODES; i++) // for every electrode excluding CZ
    {
        printf("Connecting current electrode to driven trough 1 MOhms\n");
        
        printf("//////////////////////////////////\n");
        printf("///////// Test %d  of 18 /////////\n", i+1); /// by T. Peti
        printf("//////////////////////////////////\n");
        
        if (relay_address < 75)
        {
            TI_595_state_vector[relay_address]='0';     // do not drive spst
            TI_595_state_vector[relay_address+1]='1';   // drive spdt
        }
        else
        {
            TI_595_state_vector[relay_address]='1';     // drive spdt 
            TI_595_state_vector[relay_address+1]='0';   // do not drive spst
        }
        update_595_state();
        
        // no DC
        printf("Waiting for transients to settle...\n");
        wait(TRANSIENTS_WAIT);
        printf("Waiting for CMRR measurement to finish\n");
        wait(CMRR_WAIT);
        
        kocka(0.3);     // apply +300 mV
        printf("Waiting for transients to settle...\n");
        wait(TRANSIENTS_WAIT);
        printf("Waiting for CMRR measurement to finish (+300 mV)\n");
        wait(CMRR_WAIT);
        
        kocka(-0.3);    // apply -300 mV
        printf("Waiting for transients to settle...\n");
        wait(TRANSIENTS_WAIT);
        printf("Waiting for CMRR measurement to finish (-300 mV)\n");
        wait(CMRR_WAIT);
 
        kocka(0);       // short kocka
        
        // reset relays to common
        printf("Reconnecting current electrode directly to common\n");
        if (relay_address < 75)
        {
            TI_595_state_vector[relay_address]='1';     // drive spst
            TI_595_state_vector[relay_address+1]='0';   // do not drive spdt
        }
        else
        {
            TI_595_state_vector[relay_address]='0';     // do not drive spdt 
            TI_595_state_vector[relay_address+1]='1';   // drive spst
        }
        update_595_state();     
        
        if (i<17) printf("Selecting next electrode\n");
        increment_relay_address(EXCLUDING_CZ);
    }
 
    printf("CMRR measurement finished.\n");
    
    excitacion_dc_set_voltage(0);       // clear excitation DC
    ac_dac_set(0);                      // reset excitation amplitude
    dds_set_frequency(0);               // stop DDS
    kocka(0);
    
    printf("Releasing HV_COUPLING_SPST\n");
    TI_595_state_vector[71]='0';        // release HV_COUPLING_SPST
    
    printf("Releasing COMMON_VS_MIX\n");
    TI_595_state_vector[70]='0';        // release common_vs_mix
    printf("Program exit\n");
/**** CMRR MODE END ****/
 
    return 0 ;
}