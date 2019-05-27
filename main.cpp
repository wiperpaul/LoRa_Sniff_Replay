#include "mbed.h"
#include "main.h"
#include "sx1272-hal.h"
#include "debug.h"
#include "SDFileSystem.h" // include SD filesystem for SDHC slot

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1


#define RF_FREQUENCY                                868300000 // Center EU frequencyHz
#define TX_OUTPUT_POWER                                 14        // 14 dBm

#define LORA_BANDWIDTH                              0         
// [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]

#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]

#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_FHSS_ENABLED                           false
#define LORA_NB_SYMB_HOP                            4
#define LORA_IQ_INVERSION_ON                        false
#define LORA_CRC_ENABLED                            true

#define RX_TIMEOUT_VALUE                            3500000   // in us
#define BUFFER_SIZE                                 48        // Define the payload size here

DigitalOut led(LED1);

/*
 *  Global variables declarations
 */
typedef enum {
    LOWPOWER = 0,
    IDLE,
    
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    
    TX,
    TX_TIMEOUT,

    CAD,
    CAD_DONE
} AppStates_t;

volatile AppStates_t State = LOWPOWER;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*
 *  Global variables declarations
 */
SX1272MB2xAS Radio( NULL );

uint16_t BufferSize = 0;
uint8_t Buffer[BUFFER_SIZE];

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;
bool g_repeattx = false; // global variable to decide whether or not to repeat tx ( radio.send )

SDFileSystem sd(PTE3, PTE1, PTE2, PTE4, "sd"); // MOSI, MISO, SCK, CS PINLAYOUT FOR SD CARD

int main()
{
    debug( "\n\n\r W14008354 - LoRa Receiver \n\n\r" );
    
    // Initialize Radio driver
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init( &RadioEvents );

    // verify the connection with the board
    while( Radio.Read( REG_VERSION ) == 0x00  ) {
        debug( "Radio could not be detected!\n\r", NULL );
        wait( 1 );
    }

    debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1272MB2XAS ) ) , "\n\r > Board Type: SX1272MB2xAS < \n\r" );

    Radio.SetChannel( RF_FREQUENCY );

    debug_if( LORA_FHSS_ENABLED, "\n\n\r             > LORA FHSS Mode < \n\n\r");
    debug_if( !LORA_FHSS_ENABLED, "\n\n\r             > LORA Mode < \n\n\r");
   
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                         LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                         LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                         LORA_IQ_INVERSION_ON, 2000 );
                         
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                       LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                       LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                       LORA_IQ_INVERSION_ON, true );

    debug_if( DEBUG_MESSAGE, "Starting listening loop\r\n\r\n" );

    led = 0;
    Radio.Rx( RX_TIMEOUT_VALUE );
    
    while( 1 ) {
        switch( State ) {
            case TX_TIMEOUT:
              if ( g_repeattx == true )
              {
                debug_if( DEBUG_MESSAGE, "> Repeating Tx\n\r" );
                wait_ms( 3500 ); // added a 3500ms timeout value
                Radio.Send( Buffer, BufferSize ); // radio send buffer
                State = LOWPOWER;
                break;
              }
              else
              {
                Radio.Rx( RX_TIMEOUT_VALUE ); // Sets radio in RX mode 
                State = LOWPOWER;
                break;  
              }
            case RX: // If RX case is true then the following happens if not then moves onto next case which is RX_Timeout
                if( BufferSize > 0 ) {
                    debug_if( DEBUG_MESSAGE, "\r\n========\r\nRX Recieved Packet\r\n========\r\n" );
                    // for loop which prints buffer until it hits buffersize
                    for(int i = 0; i < BufferSize; i++) {
                        // debug was missing leading zeros so used %02x to enforce 2 characters for each byte 
                        debug_if( DEBUG_MESSAGE, "%02x", Buffer[i]); 
                    } 
                    retrieve_data( Buffer ); // Runs data retrieval function which could later contain packet manipulation code
                }
                debug_if( DEBUG_MESSAGE, "\r\n========\r\nTX Radio.Send\r\n========\r\n" );
                wait_ms( 10 ); // added a 10ms wait
                Radio.Send( Buffer, BufferSize ); // radio send buffer
                State = LOWPOWER;
                break;
            case TX:
                led = !led;
                debug( "TX CASE DEBUG\r\n" );
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
            case RX_TIMEOUT:
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = LOWPOWER;
                break;
            case RX_ERROR:
                // We have received a Packet with a CRC error
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = LOWPOWER;
                break;
            case LOWPOWER:
                break;
            default:
                State = LOWPOWER;
                break;
        }
    }
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
    debug_if( DEBUG_MESSAGE, "> OnTxDone\n\r" );
    g_repeattx = false;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
    debug_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
    g_repeattx = true;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
    debug_if( DEBUG_MESSAGE, "> OnRxDone %d \n\r", RssiValue );
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    BufferSize = 0;
    State = RX_TIMEOUT;
    debug_if( DEBUG_MESSAGE, "> OnRxTimeout\n\r" );
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    debug_if( DEBUG_MESSAGE, "> OnRxError\n\r" );
}

void retrieve_data(uint8_t * payload) // Data retrieval fucntion for later manipulation 
{
}

