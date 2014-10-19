#include <string.h>

#include "stm8s.h"
#include "cc1101.h"

#define bit_set(a,b) ((a) |= (1<<(b)))
#define bit_clear(a,b) ((a) &= ~(1<<(b)))
#define bit_flip(a,b) ((a) ^= (1<<(b)))
#define bit_check(a,b) ((a) & (1<<(b)))

#define min(a, b)  (((a) < (b)) ? (a) : (b))
#define max(a, b)  (((a) > (b)) ? (a) : (b))

#define VERSION                    0

#define SS_PIND                    4
#define MISO_PINC                  7

#define MAX_CHANNELS              50
#define RX_BUFFER_SIZE            20
#define INIT_RSSI_VALID_DELAY     10

#define MESSAGE_ACK             0x01
#define MESSAGE_NACK            0x02
#define MESSAGE_WAIT            0x03

#define MESSAGE_RSSI_UPDATE     0x11
#define MESSAGE_RSSI_STORED     0x12
#define MESSAGE_VERSION         0x13

#define COMMAND_SET_MODE        0x01
#define COMMAND_SET_CHAN        0x02
#define COMMAND_SET_BANDWIDTH   0x03
#define COMMAND_SET_THRESHOLD   0x04
#define COMMAND_SET_RSSI_DELAY  0x05

#define COMMAND_CALIBRATE       0x11
#define COMMAND_RESET           0x12

#define COMMAND_GET_VERSION     0x21
#define COMMAND_GET_STORED_RSSI 0x22

#define MODE_IDLE               0x00
#define MODE_RUN                0x01
#define MODE_RUN_THRESHOLD      0x02

unsigned char rx_buffer[RX_BUFFER_SIZE];
unsigned char rx_buffer_pos = 0;
unsigned char threshold = 0;
unsigned char mode = MODE_IDLE;
unsigned char rssi_valid_delay = INIT_RSSI_VALID_DELAY;

typedef struct {
    unsigned int freq2;
    unsigned int freq1;
    unsigned int freq0;
    unsigned int fscal3;
    unsigned int fscal2;
    unsigned int fscal1;
    unsigned int rssi;
} channel_info;

channel_info chan_table[MAX_CHANNELS];

void parse_uart_rx();

void delay(unsigned long count){
    unsigned long i; 
    for(;count > 0; count--){
        for(i = 0; i < 100; i++) { 
            __asm__("nop");
        }
    }    
}

unsigned char hex(char c){
   if(c >= '0' && c <= '9')
     return (unsigned char)(c - '0');
   else
     return (unsigned char)(c-'A'+10);
}

unsigned char hex2char(const char *c, unsigned int offset){
   return hex(c[1 + offset]) + (hex(c[0 + offset]) << 4);
}

unsigned long hex2long(const char *c, unsigned int offset){
   return hex(c[7 + offset]) + (hex(c[6 + offset]) << 4)  
       + ((unsigned long)hex(c[5 + offset]) << 8)  + ((unsigned long)hex(c[4 + offset]) << 12)  
       + ((unsigned long)hex(c[3 + offset]) << 16) + ((unsigned long)hex(c[2 + offset]) << 20)  
       + ((unsigned long)hex(c[1 + offset]) << 24) + ((unsigned long)hex(c[0 + offset]) << 28);
}

int uart_write(const char *str) {
    char i;
    for(i = 0; i < strlen(str); i++) {
        while(!(UART1_SR & UART_SR_TXE));
        UART1_DR = str[i];
    }
    return(i);
}

void uart_write_byte(unsigned char c) {
    while(!(UART1_SR & UART_SR_TXE));
    UART1_DR = c;
}

void uart_write_hexbyte(unsigned char c) {
    unsigned char x;
    x=c>>4;
    while(!(UART1_SR & UART_SR_TXE));
    UART1_DR = x+(x>9?'W':'0');
    x=c&0x0f;
    while(!(UART1_SR & UART_SR_TXE));
    UART1_DR = x+(x>9?'W':'0');
}

void uart_reset_rx_buf(){
    unsigned char i;
    rx_buffer_pos = 0;
    for(i = 0; i < RX_BUFFER_SIZE; i++) rx_buffer[i] = 0;
}

void uart_recv(void) __interrupt(18){
  rx_buffer[rx_buffer_pos++] = UART1_DR;
  parse_uart_rx();
}

unsigned char spi_transfer(unsigned char value){
    SPI_DR = value;
    while (!(SPI_SR & SPI_SR_RXNE)) ;
    return SPI_DR;
}

void spi_strobe(unsigned char addr){
    bit_clear(PD_ODR, SS_PIND);
    while(bit_check(PC_IDR, MISO_PINC));
    spi_transfer(addr);
    bit_set(PD_ODR, SS_PIND);
}


unsigned char spi_read_config_register(unsigned char addr){
    unsigned char result;
    bit_clear(PD_ODR, SS_PIND);
    while(bit_check(PC_IDR, MISO_PINC));
    spi_transfer(addr|CC1101_READ_SINGLE);
    result = spi_transfer(0x00);
    bit_set(PD_ODR, SS_PIND);
    return result;
}

unsigned char spi_read_status_register(unsigned char addr){
    return spi_read_config_register(addr|CC1101_READ_BURST);
}

unsigned char spi_write_config_register(unsigned char addr, unsigned char value){
    unsigned char result;
    bit_clear(PD_ODR, SS_PIND);
    while(bit_check(PC_IDR, MISO_PINC));
    spi_transfer(addr);
    result = spi_transfer(value);
    bit_set(PD_ODR, SS_PIND);
    return result;
}


void spi_reset(){
    // CC1101 reset sequence

    bit_clear(PD_ODR, SS_PIND); delay(10); // toggle SS
    bit_set(PD_ODR, SS_PIND);   delay(10);

    bit_clear(PD_ODR, SS_PIND);          // begin SPI
    while(bit_check(PC_IDR, MISO_PINC)); // wait for SO to go low
    spi_transfer(CC1101_SRES);           // transmit reset strobe
    while(bit_check(PC_IDR, MISO_PINC)); // wait for SO to go low 
    bit_set(PD_ODR, SS_PIND);            // end SPI
}

// define channel based on frequency and calibrate
// the frequency setting is in units of 396.728515625 Hz
void setup_channel(long freq, unsigned int ch){

    if ((freq > CC1101_MIN_300 && freq < CC1101_MID_300) 
        || (freq > CC1101_MIN_400 && freq < CC1101_MID_400) 
        || (freq > CC1101_MIN_900 && freq < CC1101_MID_900)){
        spi_write_config_register(CC1101_FSCAL2, 0x0A); // LOW VCO
    }else{
        spi_write_config_register(CC1101_FSCAL2, 0x2A); // HIGH VCO
    }

    freq = freq /  396.728515625;

    spi_write_config_register(CC1101_FREQ2, (freq >> 16) & 0xff);
    spi_write_config_register(CC1101_FREQ1, (freq >> 8) & 0xff);
    spi_write_config_register(CC1101_FREQ0, freq & 0xff);   

    spi_strobe(CC1101_SCAL);
    spi_strobe(CC1101_SRX);

    // wait for calibration
    delay(rssi_valid_delay);

    // store frequency/calibration settings
    chan_table[ch].freq2 = spi_read_config_register(CC1101_FREQ2);
    chan_table[ch].freq1 = spi_read_config_register(CC1101_FREQ1);
    chan_table[ch].freq0 = spi_read_config_register(CC1101_FREQ0);
    chan_table[ch].fscal3 = spi_read_config_register(CC1101_FSCAL3);
    chan_table[ch].fscal2 = spi_read_config_register(CC1101_FSCAL2);;
    chan_table[ch].fscal1 = spi_read_config_register(CC1101_FSCAL1);;

    // get initial RSSI measurement
    chan_table[ch].rssi = spi_read_status_register(CC1101_RSSI) ^ 0x80;

    spi_strobe(CC1101_SIDLE);
    
}

unsigned char get_rssi(unsigned char ch){

    unsigned char rssi;

    spi_write_config_register(CC1101_FREQ2, chan_table[ch].freq2);
    spi_write_config_register(CC1101_FREQ1, chan_table[ch].freq1);
    spi_write_config_register(CC1101_FREQ0, chan_table[ch].freq0);

    spi_write_config_register(CC1101_FSCAL3, chan_table[ch].fscal3);
    spi_write_config_register(CC1101_FSCAL2, chan_table[ch].fscal2);
    spi_write_config_register(CC1101_FSCAL1, chan_table[ch].fscal1);
    
    spi_strobe(CC1101_SRX);
    delay(rssi_valid_delay);
    rssi = spi_read_status_register(CC1101_RSSI) ^ 0x80;
    spi_strobe(CC1101_SIDLE);
    return rssi;
}

void set_bandwidth(unsigned char bandwidth){
    // in khz from 0x00 to 0x0F
    // 58, 68, 81, 102, 116, 135, 162, 203, 
    // 232, 270, 325, 406, 464, 541, 650, 812
    spi_write_config_register(CC1101_MDMCFG4, ((0xFF-bandwidth) << 4) | 0x0C);
}

void rssi_fine_calibration(unsigned char count){
    unsigned char ch;
    unsigned long rssi;
    unsigned char i;
    for(ch = 0; ch < MAX_CHANNELS; ch++){
        rssi = 0;
        for(i = 0; i < count; i++){
            rssi += get_rssi(ch);
        }
        chan_table[ch].rssi = rssi / count; 
    }
}


void init_channels(){
    unsigned char ch;
    for(ch = 0; ch < MAX_CHANNELS; ch++){
        chan_table[ch].freq2 = 0x80; // sentinel value, unconfigured
    }
}

void init_spi(){

    // reset the CC1101
    spi_reset();

    // IF of 457.031 kHz
    spi_write_config_register(CC1101_FSCTRL1, 0x12);
    spi_write_config_register(CC1101_FSCTRL0, 0x00);

    // disable 3 highest DVGA settings
    spi_write_config_register(CC1101_AGCCTRL2, 0xC3);

    // frequency synthesizer calibration 
    spi_write_config_register(CC1101_FSCAL3, 0xEA);
    spi_write_config_register(CC1101_FSCAL2, 0x2A);
    spi_write_config_register(CC1101_FSCAL1, 0x00);
    spi_write_config_register(CC1101_FSCAL0, 0x1F);

    // "various test settings" 
    spi_write_config_register(CC1101_TEST2, 0x88);
    spi_write_config_register(CC1101_TEST1, 0x31);
    spi_write_config_register(CC1101_TEST0, 0x09);

    // no automatic frequency calibration 
    spi_write_config_register(CC1101_MCSM0, 0x00);

}

void init(){
    CLK_CKDIVR = 0x00; // 16 MHz clock

    // GPIO setup
    bit_set(PD_DDR, SS_PIND);
    bit_set(PD_CR1, SS_PIND);
    bit_set(PD_ODR, SS_PIND);

    // UART
    UART1_CR2 = UART_CR2_TEN | UART_CR2_REN | UART_CR2_RIEN; // enable tx and rx, set rx as interrupt
    UART1_CR3 &= ~(UART_CR3_STOP1 | UART_CR3_STOP2); // 1 stop bit
    //UART1_BRR2 = 0x03; UART1_BRR1 = 0x68; // 9600 baud
    UART1_BRR2 = 0x0a; UART1_BRR1 = 0x08; // 115200

    // SPI
    SPI_CR1 = 0x4C; // 0100 1100 MSB first, SPI enabled, 4 MHz clock, Master mode, CPOL=0, CPHA=0

    uart_reset_rx_buf();
    
    // INTERRUPT
    __asm__("rim\n"); // enable interrupts

}

void parse_uart_rx(){
    unsigned char len;
    unsigned char op;
    unsigned char ch;
    unsigned char count;
    unsigned char bandwidth;
    unsigned long freq;

    if(rx_buffer_pos >= RX_BUFFER_SIZE){
        uart_reset_rx_buf();
        return;
    }

    len = strlen(rx_buffer);

    if(rx_buffer[len-2] == '\r' && rx_buffer[len-1] == '\n'){

        op = hex2char(rx_buffer, 0);

        switch (op) {
            case COMMAND_SET_CHAN:
                ch = hex2char(rx_buffer, 2);
                freq = hex2long(rx_buffer, 4);
                setup_channel(freq, ch); 
                uart_write_hexbyte(MESSAGE_ACK);
                uart_write("\r\n");
                break;

            case COMMAND_SET_THRESHOLD: 
                threshold = hex2char(rx_buffer, 2);
                uart_write_hexbyte(MESSAGE_ACK);
                uart_write("\r\n");
                break;

            case COMMAND_CALIBRATE: 
                count = hex2char(rx_buffer, 2);
                uart_write_hexbyte(MESSAGE_WAIT);
                uart_write("\r\n");
                rssi_fine_calibration(count);
                uart_write_hexbyte(MESSAGE_ACK);
                uart_write("\r\n");
                break;

            case COMMAND_RESET: 
                init_spi();
                init_channels();
                uart_write_hexbyte(MESSAGE_ACK);
                uart_write("\r\n");
                break;

            case COMMAND_SET_MODE: 
                mode = hex2char(rx_buffer, 2);
                uart_write_hexbyte(MESSAGE_ACK);
                uart_write("\r\n");
                break;

            case COMMAND_SET_BANDWIDTH: 
                bandwidth = hex2char(rx_buffer, 2);
                set_bandwidth(bandwidth);
                uart_write_hexbyte(MESSAGE_ACK);
                uart_write("\r\n");
                break;

            case COMMAND_SET_RSSI_DELAY:
                rssi_valid_delay = hex2char(rx_buffer, 2);
                uart_write_hexbyte(MESSAGE_ACK);
                uart_write("\r\n");
                break;

            case COMMAND_GET_STORED_RSSI: 
                ch = hex2char(rx_buffer, 2);
                uart_write_hexbyte(MESSAGE_RSSI_STORED);
                uart_write_hexbyte(chan_table[ch].rssi);
                uart_write("\r\n");
                break;

            case COMMAND_GET_VERSION:
                uart_write_hexbyte(MESSAGE_VERSION);
                uart_write_hexbyte(VERSION);
                uart_write("\r\n");
                break;
        }

        uart_reset_rx_buf();
    }
}

int main() {
    unsigned char ch;
    unsigned char rssi;

    init();
    init_spi();
    init_channels();

    while(1){

        if(mode){
            for(ch = 0; ch < MAX_CHANNELS; ch++){
                if(!(chan_table[ch].freq2 & 0x80)){
                    rssi = get_rssi(ch);
                    if(mode == MODE_RUN || (mode == MODE_RUN_THRESHOLD && (rssi > (chan_table[ch].rssi + threshold)))){
                        uart_write_hexbyte(MESSAGE_RSSI_UPDATE);
                        uart_write_hexbyte(ch);
                        uart_write_hexbyte(rssi);
                        uart_write_byte('\r');
                        uart_write_byte('\n');
                    }
                }
            }
        }  
    }
}
    