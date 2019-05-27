/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C) 2015 Semtech

Description: -

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/
#include "sx1272-hal.h"

const RadioRegisters_t SX1272MB2xAS::RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

SX1272MB2xAS::SX1272MB2xAS( RadioEvents_t *events,
                            PinName mosi, PinName miso, PinName sclk, PinName nss, PinName reset,
                            PinName dio0, PinName dio1, PinName dio2, PinName dio3, PinName dio4, PinName dio5,
#if defined ( TARGET_MOTE_L152RC )
                            PinName rfSwitchCntr1, PinName rfSwitchCntr2 )
#elif defined ( TARGET_MTS_MDOT_F411RE )
                            PinName txctl, PinName rxctl )
#else
                            PinName antSwitch )
#endif
                            : SX1272( events, mosi, miso, sclk, nss, reset, dio0, dio1, dio2, dio3, dio4, dio5 ),
#if defined ( TARGET_MOTE_L152RC )
                            RfSwitchCntr1( rfSwitchCntr1 ),
                            RfSwitchCntr2( rfSwitchCntr2 ),
                            PwrAmpCntr( PD_2 )               
#elif defined ( TARGET_MTS_MDOT_F411RE )
                            TxCtl ( txctl ),
                            RxCtl ( rxctl )
#else
                            AntSwitch( antSwitch ),
                        #if( defined ( TARGET_NUCLEO_L152RE ) ) || defined ( TARGET_NUCLEO_L476RG )
                            Fake( D8 )
                        #else
                            Fake( A3 )
                        #endif
#endif
{
    this->RadioEvents = events;

    Reset( );

    IoInit( );

    SetOpMode( RF_OPMODE_SLEEP );

    IoIrqInit( dioIrq );

    RadioRegistersInit( );

    SetModem( MODEM_FSK );

    this->settings.State = RF_IDLE ;
}

SX1272MB2xAS::SX1272MB2xAS( RadioEvents_t *events )
                        #if defined ( TARGET_NUCLEO_L152RE ) || defined ( TARGET_NUCLEO_L476RG )
                        :   SX1272( events, D11, D12, D13, D10, A0, D2, D3, D4, D5, A3, D9 ), // For NUCLEO L152RE dio4 is on port A3
                            AntSwitch( A4 ),
                            Fake( D8 )
                        #elif defined ( TARGET_MOTE_L152RC )
                        :   SX1272( events, PB_15, PB_14, PB_13, PB_12, PC_2, PC_6, PC_10, PC_11, PC_8, PC_9, PC_12 ),
                            RfSwitchCntr1( PC_4 ),
                            RfSwitchCntr2( PC_13 ),
                            PwrAmpCntr( PD_2 )
                        #elif defined ( TARGET_MTS_MDOT_F411RE )
                        :   SX1272( events, LORA_MOSI, LORA_MISO, LORA_SCK, LORA_NSS, LORA_RESET, LORA_DIO0, LORA_DIO1, LORA_DIO2, LORA_DIO3, LORA_DIO4, LORA_DIO5 ),
                            TxCtl( LORA_TXCTL ),
                            RxCtl( LORA_RXCTL )
                        #else
                        :   SX1272( events, D11, D12, D13, D10, A0, D2, D3, D4, D5, D8, D9 ),
                            AntSwitch( A4 ), 
                            Fake( A3 )
                        #endif
{
    this->RadioEvents = events;

    Reset( );

    boardConnected = UNKNOWN;

    DetectBoardType( );

    IoInit( );

    SetOpMode( RF_OPMODE_SLEEP );
    IoIrqInit( dioIrq );

    RadioRegistersInit( );

    SetModem( MODEM_FSK );

    this->settings.State = RF_IDLE ;
}

//-------------------------------------------------------------------------
//                      Board relative functions
//-------------------------------------------------------------------------
uint8_t SX1272MB2xAS::DetectBoardType( void )
{
    if( boardConnected == UNKNOWN )
    {
#if defined ( TARGET_MOTE_L152RC )
        boardConnected = NA_MOTE_72;
#elif defined ( TARGET_MTS_MDOT_F411RE )
        boardConnected = MDOT_F411RE;
#else
        this->AntSwitch.input( );
        wait_ms( 1 );
        if( this->AntSwitch == 1 )
        {
            boardConnected = SX1272MB1DCS;
        }
        else
        {
            boardConnected = SX1272MB2XAS;
        }
        this->AntSwitch.output( );
        wait_ms( 1 );
#endif
    }
    return ( boardConnected );
}

void SX1272MB2xAS::IoInit( void )
{
    AntSwInit( );
    SpiInit( );
}

void SX1272MB2xAS::RadioRegistersInit( )
{
    uint8_t i = 0;
    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SetModem( RadioRegsInit[i].Modem );
        Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }    
}

void SX1272MB2xAS::SpiInit( void )
{
    nss = 1;    
    spi.format( 8,0 );   
    uint32_t frequencyToSet = 8000000;
    #if( defined ( TARGET_NUCLEO_L152RE ) || defined ( TARGET_MOTE_L152RC ) || defined ( TARGET_NUCLEO_L476RG ) ||  defined ( TARGET_LPC11U6X ) || defined ( TARGET_MTS_MDOT_F411RE ) )
        spi.frequency( frequencyToSet );
    #elif( defined ( TARGET_KL25Z ) ) //busclock frequency is halved -> double the spi frequency to compensate
        spi.frequency( frequencyToSet * 2 );
    #else
        #warning "Check the board's SPI frequency"
    #endif
    wait(0.1); 
}

void SX1272MB2xAS::IoIrqInit( DioIrqHandler *irqHandlers )
{
#if( defined ( TARGET_NUCLEO_L152RE ) || defined ( TARGET_MOTE_L152RC ) || defined ( TARGET_NUCLEO_L476RG ) || defined ( TARGET_NUCLEO_L476RG ) ||  defined ( TARGET_LPC11U6X ) )
    dio0.mode( PullDown );
    dio1.mode( PullDown );
    dio2.mode( PullDown );
    dio3.mode( PullDown );
    dio4.mode( PullDown );
#endif
    dio0.rise( this, static_cast< TriggerMB2xAS > ( irqHandlers[0] ) );
    dio1.rise( this, static_cast< TriggerMB2xAS > ( irqHandlers[1] ) );
    dio2.rise( this, static_cast< TriggerMB2xAS > ( irqHandlers[2] ) );
    dio3.rise( this, static_cast< TriggerMB2xAS > ( irqHandlers[3] ) );
    dio4.rise( this, static_cast< TriggerMB2xAS > ( irqHandlers[4] ) );
}

void SX1272MB2xAS::IoDeInit( void )
{
    //nothing
}

uint8_t SX1272MB2xAS::GetPaSelect( uint32_t channel )
{
    if( boardConnected == SX1272MB1DCS || boardConnected == MDOT_F411RE )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1272MB2xAS::SetAntSwLowPower( bool status )
{
    if( isRadioActive != status )
    {
        isRadioActive = status;
    
        if( status == false )
        {
            AntSwInit( );
        }
        else
        {
            AntSwDeInit( );
        }
    }
}

void SX1272MB2xAS::AntSwInit( void )
{
#if defined ( TARGET_MOTE_L152RC )
    this->RfSwitchCntr1 = 0;
    this->RfSwitchCntr2 = 0;
    this->PwrAmpCntr = 0;
#elif defined ( TARGET_MTS_MDOT_F411RE )
    this->TxCtl = 0;
    this->RxCtl = 0;
#else
    this->AntSwitch = 0;
#endif
}

void SX1272MB2xAS::AntSwDeInit( void )
{
#if defined ( TARGET_MOTE_L152RC )
    this->RfSwitchCntr1 = 0;
    this->RfSwitchCntr2 = 0;
    this->PwrAmpCntr = 0;
#elif defined ( TARGET_MTS_MDOT_F411RE )
    this->TxCtl = 0;
    this->RxCtl = 0;    
#else
    this->AntSwitch = 0;
#endif
}

void SX1272MB2xAS::SetAntSw( uint8_t rxTx )
{
#if defined ( TARGET_MOTE_L152RC )
    switch( this->currentOpMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        if( ( Read( REG_PACONFIG ) & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
        {
            this->RfSwitchCntr1 = 1;
            this->RfSwitchCntr2 = 0;
        }
        else
        {
            this->RfSwitchCntr1 = 0;
            this->RfSwitchCntr2 = 1;
        }
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
        this->RfSwitchCntr1 = 1;
        this->RfSwitchCntr2 = 1;
        break;
    default:
        this->RfSwitchCntr1 = 0;
        this->RfSwitchCntr2 = 0;
        this->PwrAmpCntr = 0;
        break;
    }
#elif defined ( TARGET_MTS_MDOT_F411RE )
    /* SKY13350 */
    this->rxTx = rxTx;

    // 1: Tx, 0: Rx
    if( rxTx != 0 )
    {
        this->TxCtl = 1;
        this->RxCtl = 0;   
    }
    else
    {
        this->TxCtl = 0;
        this->RxCtl = 1;   
    }    
#else
    this->rxTx = rxTx;

    // 1: Tx, 0: Rx
    if( rxTx != 0 )
    {
        this->AntSwitch = 1;
    }
    else
    {
        this->AntSwitch = 0;
    }
#endif
}

bool SX1272MB2xAS::CheckRfFrequency( uint32_t frequency )
{
    //TODO: Implement check, currently all frequencies are supported
    return true;
}


void SX1272MB2xAS::Reset( void )
{
    reset.output();
    reset = 0;
    wait_ms( 1 );
    reset.input();
    wait_ms( 6 );
}
    
void SX1272MB2xAS::Write( uint8_t addr, uint8_t data )
{
    Write( addr, &data, 1 );
}

uint8_t SX1272MB2xAS::Read( uint8_t addr )
{
    uint8_t data;
    Read( addr, &data, 1 );
    return data;
}

void SX1272MB2xAS::Write( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    nss = 0;
    spi.write( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        spi.write( buffer[i] );
    }
    nss = 1;
}

void SX1272MB2xAS::Read( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    nss = 0;
    spi.write( addr & 0x7F );
    for( i = 0; i < size; i++ )
    {
        buffer[i] = spi.write( 0 );
    }
    nss = 1;
}

void SX1272MB2xAS::WriteFifo( uint8_t *buffer, uint8_t size )
{
    Write( 0, buffer, size );
}

void SX1272MB2xAS::ReadFifo( uint8_t *buffer, uint8_t size )
{
    Read( 0, buffer, size );
}
