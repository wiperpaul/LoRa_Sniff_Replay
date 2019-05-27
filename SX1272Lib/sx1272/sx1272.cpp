/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C) 2015 Semtech

Description: Actual implementation of a SX1272 radio, inherits Radio

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/
#include "sx1272.h"

const FskBandwidth_t SX1272::FskBandwidths[] =
{       
    { 2600  , 0x17 },   
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Badwidth
};


SX1272::SX1272( RadioEvents_t *events,
                PinName mosi, PinName miso, PinName sclk, PinName nss, PinName reset,
                PinName dio0, PinName dio1, PinName dio2, PinName dio3, PinName dio4, PinName dio5 )
            :   Radio( events ),
                spi( mosi, miso, sclk ),
                nss( nss ),
                reset( reset ),
                dio0( dio0 ), dio1( dio1 ), dio2( dio2 ), dio3( dio3 ), dio4( dio4 ), dio5( dio5 ),
                isRadioActive( false )
{
    wait_ms( 10 );
    this->rxTx = 0;
    this->rxtxBuffer = new uint8_t[RX_BUFFER_SIZE];
    currentOpMode = RF_OPMODE_STANDBY;
    
    this->RadioEvents = events;
    
    this->dioIrq = new DioIrqHandler[6];

    this->dioIrq[0] = &SX1272::OnDio0Irq;
    this->dioIrq[1] = &SX1272::OnDio1Irq;
    this->dioIrq[2] = &SX1272::OnDio2Irq;
    this->dioIrq[3] = &SX1272::OnDio3Irq;
    this->dioIrq[4] = &SX1272::OnDio4Irq;
    this->dioIrq[5] = NULL;
    
    this->settings.State = RF_IDLE;
}

SX1272::~SX1272( )
{
    delete this->rxtxBuffer;
    delete this->dioIrq;
}

void SX1272::Init( RadioEvents_t *events )
{
    this->RadioEvents = events;
}

RadioState SX1272::GetStatus( void )
{
    return this->settings.State;
}

void SX1272::SetChannel( uint32_t freq )
{
    this->settings.Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}

bool SX1272::IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh )
{
    int16_t rssi = 0;

    SetModem( modem );

    SetChannel( freq );

    SetOpMode( RF_OPMODE_RECEIVER );

    wait_ms( 1 );

    rssi = GetRssi( modem );

    Sleep( );

    if( rssi > rssiThresh )
    {
        return false;
    }
    return true;
}

uint32_t SX1272::Random( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    SetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SetOpMode( RF_OPMODE_RECEIVER );

    for( i = 0; i < 32; i++ )
    {
        wait_ms( 1 );
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )Read( REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }

    Sleep( );

    return rnd;
}

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
uint8_t SX1272::GetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

void SX1272::SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        {
            this->settings.Fsk.Bandwidth = bandwidth;
            this->settings.Fsk.Datarate = datarate;
            this->settings.Fsk.BandwidthAfc = bandwidthAfc;
            this->settings.Fsk.FixLen = fixLen;
            this->settings.Fsk.PayloadLen = payloadLen;
            this->settings.Fsk.CrcOn = crcOn;
            this->settings.Fsk.IqInverted = iqInverted;
            this->settings.Fsk.RxContinuous = rxContinuous;
            this->settings.Fsk.PreambleLen = preambleLen;

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            Write( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            Write( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            Write( REG_RXBW, GetFskBandwidthRegValue( bandwidth ) );
            Write( REG_AFCBW, GetFskBandwidthRegValue( bandwidthAfc ) );

            Write( REG_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            Write( REG_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                Write( REG_PAYLOADLENGTH, payloadLen );
            }
            else
            {
                Write( REG_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum
            }
            
            Write( REG_PACKETCONFIG1,
                         ( Read( REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
        }
        break;
    case MODEM_LORA:
        {
            this->settings.LoRa.Bandwidth = bandwidth;
            this->settings.LoRa.Datarate = datarate;
            this->settings.LoRa.Coderate = coderate;
            this->settings.LoRa.PreambleLen = preambleLen;
            this->settings.LoRa.FixLen = fixLen;
            this->settings.LoRa.PayloadLen = payloadLen;
            this->settings.LoRa.CrcOn = crcOn;
            this->settings.LoRa.FreqHopOn = freqHopOn;
            this->settings.LoRa.HopPeriod = hopPeriod;
            this->settings.LoRa.IqInverted = iqInverted;
            this->settings.LoRa.RxContinuous = rxContinuous;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                this->settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                this->settings.LoRa.LowDatarateOptimize = 0x00;
            }

            Write( REG_LR_MODEMCONFIG1,
                         ( Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
                           RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK ) |
                           ( bandwidth << 6 ) | ( coderate << 3 ) |
                           ( fixLen << 2 ) | ( crcOn << 1 ) |
                           this->settings.LoRa.LowDatarateOptimize );

            Write( REG_LR_MODEMCONFIG2,
                         ( Read( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

            Write( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

            Write( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            Write( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                Write( REG_LR_PAYLOADLENGTH, payloadLen );
            }

            if( this->settings.LoRa.FreqHopOn == true )
            {
                Write( REG_LR_PLLHOP, ( Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                Write( REG_LR_HOPPERIOD, this->settings.LoRa.HopPeriod );
            }

            if( datarate == 6 )
            {
                Write( REG_LR_DETECTOPTIMIZE,
                             ( Read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                Write( REG_LR_DETECTOPTIMIZE,
                             ( Read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}
#if defined ( TARGET_MOTE_L152RC )
/*     PD_2=0  PD_2=1
op PaB  rfo     rfo
0  4.6  18.5    27.0
1  5.6  21.1    28.1
2  6.7  23.3    29.1
3  7.7  25.3    30.1
4  8.8  26.2    30.7
5  9.8  27.3    31.2
6  10.7 28.1    31.6
7  11.7 28.6    32.2
8  12.8 29.2    32.4
9  13.7 29.9    32.9
10 14.7 30.5    33.1
11 15.6 30.8    33.4
12 16.4 30.9    33.6
13 17.1 31.0    33.7
14 17.8 31.1    33.7
15 18.4 31.1    33.7
*/
//                           txpow:   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14  15  16  17  18  19
static const uint8_t PaBTable[20] = { 0, 0, 0, 0, 0, 1, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15 };

//                           txpow:  20 21 22 23 24 25 26 27 28 29 30
static const uint8_t RfoTable[11] = { 1, 1, 1, 2, 2, 3, 4, 5, 6, 8, 9 };
#endif

void SX1272::SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    SetModem( modem );

    paConfig = Read( REG_PACONFIG );
    paDac = Read( REG_PADAC );

#if defined ( TARGET_MOTE_L152RC )
    if( power > 19 )
    {
        paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_RFO;
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | RfoTable[power - 20];
    }
    else
    {
        paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_PABOOST;
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | PaBTable[power];
    }
#else
    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | GetPaSelect( this->settings.Channel );

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
#endif
    Write( REG_PACONFIG, paConfig );
    Write( REG_PADAC, paDac );

    switch( modem )
    {
    case MODEM_FSK:
        {
            this->settings.Fsk.Power = power;
            this->settings.Fsk.Fdev = fdev;
            this->settings.Fsk.Bandwidth = bandwidth;
            this->settings.Fsk.Datarate = datarate;
            this->settings.Fsk.PreambleLen = preambleLen;
            this->settings.Fsk.FixLen = fixLen;
            this->settings.Fsk.CrcOn = crcOn;
            this->settings.Fsk.IqInverted = iqInverted;
            this->settings.Fsk.TxTimeout = timeout;

            fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
            Write( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
            Write( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            Write( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            Write( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            Write( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            Write( REG_PREAMBLELSB, preambleLen & 0xFF );

            Write( REG_PACKETCONFIG1,
                         ( Read( REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );

        }
        break;
    case MODEM_LORA:
        {
            this->settings.LoRa.Power = power;
            this->settings.LoRa.Bandwidth = bandwidth;
            this->settings.LoRa.Datarate = datarate;
            this->settings.LoRa.Coderate = coderate;
            this->settings.LoRa.PreambleLen = preambleLen;
            this->settings.LoRa.FixLen = fixLen;
            this->settings.LoRa.FreqHopOn = freqHopOn;
            this->settings.LoRa.HopPeriod = hopPeriod;
            this->settings.LoRa.CrcOn = crcOn;
            this->settings.LoRa.IqInverted = iqInverted;
            this->settings.LoRa.TxTimeout = timeout;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                this->settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                this->settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if( this->settings.LoRa.FreqHopOn == true )
            {
                Write( REG_LR_PLLHOP, ( Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                Write( REG_LR_HOPPERIOD, this->settings.LoRa.HopPeriod );
            }

            Write( REG_LR_MODEMCONFIG1,
                         ( Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
                           RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK ) |
                           ( bandwidth << 6 ) | ( coderate << 3 ) |
                           ( fixLen << 2 ) | ( crcOn << 1 ) |
                           this->settings.LoRa.LowDatarateOptimize );

            Write( REG_LR_MODEMCONFIG2,
                        ( Read( REG_LR_MODEMCONFIG2 ) &
                          RFLR_MODEMCONFIG2_SF_MASK ) |
                          ( datarate << 4 ) );


            Write( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            Write( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

            if( datarate == 6 )
            {
                Write( REG_LR_DETECTOPTIMIZE,
                             ( Read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                Write( REG_LR_DETECTOPTIMIZE,
                             ( Read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

double SX1272::TimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    uint32_t airTime = 0;

    switch( modem )
    {
    case MODEM_FSK:
        {
            airTime = rint( ( 8 * ( this->settings.Fsk.PreambleLen +
                                     ( ( Read( REG_SYNCCONFIG ) & ~RF_SYNCCONFIG_SYNCSIZE_MASK ) + 1 ) +
                                     ( ( this->settings.Fsk.FixLen == 0x01 ) ? 0.0 : 1.0 ) +
                                     ( ( ( Read( REG_PACKETCONFIG1 ) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK ) != 0x00 ) ? 1.0 : 0 ) +
                                     pktLen +
                                     ( ( this->settings.Fsk.CrcOn == 0x01 ) ? 2.0 : 0 ) ) /
                                     this->settings.Fsk.Datarate ) * 1e6 );
        }
        break;
    case MODEM_LORA:
        {
            double bw = 0.0;
            switch( this->settings.LoRa.Bandwidth )
            {
            case 0: // 125 kHz
                bw = 125e3;
                break;
            case 1: // 250 kHz
                bw = 250e3;
                break;
            case 2: // 500 kHz
                bw = 500e3;
                break;
            }

            // Symbol rate : time for one symbol (secs)
            double rs = bw / ( 1 << this->settings.LoRa.Datarate );
            double ts = 1 / rs;
            // time of preamble
            double tPreamble = ( this->settings.LoRa.PreambleLen + 4.25 ) * ts;
            // Symbol length of payload and time
            double tmp = ceil( ( 8 * pktLen - 4 * this->settings.LoRa.Datarate +
                                 28 + 16 * this->settings.LoRa.CrcOn -
                                 ( this->settings.LoRa.FixLen ? 20 : 0 ) ) /
                                 ( double )( 4 * this->settings.LoRa.Datarate -
                                 ( ( this->settings.LoRa.LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) *
                                 ( this->settings.LoRa.Coderate + 4 );
            double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
            double tPayload = nPayload * ts;
            // Time on air
            double tOnAir = tPreamble + tPayload;
            // return us secs
            airTime = floor( tOnAir * 1e6 + 0.999 );
        }
        break;
    }
    return airTime;
}

void SX1272::Send( uint8_t *buffer, uint8_t size )
{
    uint32_t txTimeout = 0;

    switch( this->settings.Modem )
    {
    case MODEM_FSK:
        {
            this->settings.FskPacketHandler.NbBytes = 0;
            this->settings.FskPacketHandler.Size = size;

            if( this->settings.Fsk.FixLen == false )
            {
                WriteFifo( ( uint8_t* )&size, 1 );
            }
            else
            {
                Write( REG_PAYLOADLENGTH, size );
            }

            if( ( size > 0 ) && ( size <= 64 ) )
            {
                this->settings.FskPacketHandler.ChunkSize = size;
            }
            else
            {
                memcpy( rxtxBuffer, buffer, size );
                this->settings.FskPacketHandler.ChunkSize = 32;
            }

            // Write payload buffer
            WriteFifo( buffer, this->settings.FskPacketHandler.ChunkSize );
            this->settings.FskPacketHandler.NbBytes += this->settings.FskPacketHandler.ChunkSize;
            txTimeout = this->settings.Fsk.TxTimeout;
        }
        break;
    case MODEM_LORA:
        {
            if( this->settings.LoRa.IqInverted == true )
            {
                Write( REG_LR_INVERTIQ, ( ( Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                Write( REG_LR_INVERTIQ, ( ( Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            this->settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            Write( REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx
            Write( REG_LR_FIFOTXBASEADDR, 0 );
            Write( REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            if( ( Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
            {
                Standby( );
                wait_ms( 1 );
            }
            // Write payload buffer
            WriteFifo( buffer, size );
            txTimeout = this->settings.LoRa.TxTimeout;
        }
        break;
    }

    Tx( txTimeout );
}

void SX1272::Sleep( void )
{
    txTimeoutTimer.detach(  );
    rxTimeoutTimer.detach( );

    SetOpMode( RF_OPMODE_SLEEP );
    this->settings.State = RF_IDLE;
}

void SX1272::Standby( void )
{
    txTimeoutTimer.detach(  );
    rxTimeoutTimer.detach( );

    SetOpMode( RF_OPMODE_STANDBY );
    this->settings.State = RF_IDLE;
}

void SX1272::Rx( uint32_t timeout )
{
    bool rxContinuous = false;
    
    switch( this->settings.Modem )
    {
    case MODEM_FSK:
        {
            rxContinuous = this->settings.Fsk.RxContinuous;

            // DIO0=PayloadReady
            // DIO1=FifoLevel
            // DIO2=SyncAddr
            // DIO3=FifoEmpty
            // DIO4=Preamble
            // DIO5=ModeReady
            Write( REG_DIOMAPPING1, ( Read( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO0_00 |
                                                                            RF_DIOMAPPING1_DIO1_00 |
                                                                            RF_DIOMAPPING1_DIO2_11 );

            Write( REG_DIOMAPPING2, ( Read( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) |
                                                                            RF_DIOMAPPING2_DIO4_11 |
                                                                            RF_DIOMAPPING2_MAP_PREAMBLEDETECT );

            this->settings.FskPacketHandler.FifoThresh = Read( REG_FIFOTHRESH ) & 0x3F;

            Write( REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT );

            this->settings.FskPacketHandler.PreambleDetected = false;
            this->settings.FskPacketHandler.SyncWordDetected = false;
            this->settings.FskPacketHandler.NbBytes = 0;
            this->settings.FskPacketHandler.Size = 0;
        }
        break;
    case MODEM_LORA:
        {
            if( this->settings.LoRa.IqInverted == true )
            {
                Write( REG_LR_INVERTIQ, ( ( Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                Write( REG_LR_INVERTIQ, ( ( Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            rxContinuous = this->settings.LoRa.RxContinuous;

            if( this->settings.LoRa.FreqHopOn == true )
            {
                Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone, DIO2=FhssChangeChannel
                Write( REG_DIOMAPPING1, ( Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone
                Write( REG_DIOMAPPING1, ( Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            Write( REG_LR_FIFORXBASEADDR, 0 );
            Write( REG_LR_FIFOADDRPTR, 0 );
        }
        break;
    }

    memset( rxtxBuffer, 0, ( size_t )RX_BUFFER_SIZE );

    this->settings.State = RF_RX_RUNNING;
    if( timeout != 0 )
    {
        rxTimeoutTimer.attach_us( this, &SX1272::OnTimeoutIrq, timeout );
    }

    if( this->settings.Modem == MODEM_FSK )
    {
        SetOpMode( RF_OPMODE_RECEIVER );

        if( rxContinuous == false )
        {
            rxTimeoutSyncWord.attach_us( this, &SX1272::OnTimeoutIrq, ceil( ( 8.0 * ( this->settings.Fsk.PreambleLen +
                                                             ( ( Read( REG_SYNCCONFIG ) &
                                                                ~RF_SYNCCONFIG_SYNCSIZE_MASK ) +
                                                                1.0 ) + 10.0 ) /
                                                             ( double )this->settings.Fsk.Datarate ) * 1e6 ) + 4000 );
        }
    }
    else
    {
        if( rxContinuous == true )
        {
            SetOpMode( RFLR_OPMODE_RECEIVER );
        }
        else
        {
            SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
    }
}

void SX1272::Tx( uint32_t timeout )
{

    switch( this->settings.Modem )
    {
    case MODEM_FSK:
        {
            // DIO0=PacketSent
            // DIO1=FifoEmpty
            // DIO2=FifoFull
            // DIO3=FifoEmpty
            // DIO4=LowBat
            // DIO5=ModeReady
            Write( REG_DIOMAPPING1, ( Read( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO1_01 );

            Write( REG_DIOMAPPING2, ( Read( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) );
            this->settings.FskPacketHandler.FifoThresh = Read( REG_FIFOTHRESH ) & 0x3F;
        }
        break;
    case MODEM_LORA:
        {
            if( this->settings.LoRa.FreqHopOn == true )
            {
                Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                Write( REG_DIOMAPPING1, ( Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                Write( REG_DIOMAPPING1, ( Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }

    this->settings.State = RF_TX_RUNNING;
    txTimeoutTimer.attach_us( this, &SX1272::OnTimeoutIrq, timeout );
    SetOpMode( RF_OPMODE_TRANSMITTER );
}

void SX1272::StartCad( void )
{
    switch( this->settings.Modem )
    {
    case MODEM_FSK:
        {

        }
        break;
    case MODEM_LORA:
        {
            Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED
                                        );

            // DIO3=CADDone
            Write( REG_DIOMAPPING1, ( Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );

            this->settings.State = RF_CAD;
            SetOpMode( RFLR_OPMODE_CAD );
        }
        break;
    default:
        break;
    }
}

int16_t SX1272::GetRssi( RadioModems_t modem )
{
    int16_t rssi = 0;

    switch( modem )
    {
    case MODEM_FSK:
        rssi = -( Read( REG_RSSIVALUE ) >> 1 );
        break;
    case MODEM_LORA:
        rssi = RSSI_OFFSET + Read( REG_LR_RSSIVALUE );
        break;
    default:
        rssi = -1;
        break;
    }
    return rssi;
}

void SX1272::SetOpMode( uint8_t opMode )
{
    currentOpMode = opMode; // Update global variable
    
    if( opMode == RF_OPMODE_SLEEP )
    {
        SetAntSwLowPower( true );
    }
    else
    {
        SetAntSwLowPower( false );
        if( opMode == RF_OPMODE_TRANSMITTER )
        {
            SetAntSw( 1 );
        }
        else
        {
            SetAntSw( 0 );
        }
    }
    Write( REG_OPMODE, ( Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

void SX1272::SetModem( RadioModems_t modem )
{
    if( this->settings.Modem == modem )
    {
        return;
    }

    this->settings.Modem = modem;
    switch( this->settings.Modem )
    {
    default:
    case MODEM_FSK:
        SetOpMode( RF_OPMODE_SLEEP );
        Write( REG_OPMODE, ( Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );
    
        Write( REG_DIOMAPPING1, 0x00 );
        Write( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SetOpMode( RF_OPMODE_SLEEP );
        Write( REG_OPMODE, ( Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

        Write( REG_DIOMAPPING1, 0x00 );
        Write( REG_DIOMAPPING2, 0x00 );
        break;
    }
}

void SX1272::SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    this->SetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        if( this->settings.Fsk.FixLen == false )
        {
            this->Write( REG_PAYLOADLENGTH, max );
        }
        break;
    case MODEM_LORA:
        this->Write( REG_LR_PAYLOADMAXLENGTH, max );
        break;
    }
}

void SX1272::OnTimeoutIrq( void )
{
    switch( this->settings.State )
    {
    case RF_RX_RUNNING:
        if( this->settings.Modem == MODEM_FSK )
        {
            this->settings.FskPacketHandler.PreambleDetected = false;
            this->settings.FskPacketHandler.SyncWordDetected = false;
            this->settings.FskPacketHandler.NbBytes = 0;
            this->settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            Write( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                        RF_IRQFLAGS1_SYNCADDRESSMATCH );
            Write( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

            if( this->settings.Fsk.RxContinuous == true )
            {
                // Continuous mode restart Rx chain
                Write( REG_RXCONFIG, Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                rxTimeoutSyncWord.attach_us( this, &SX1272::OnTimeoutIrq, ceil( ( 8.0 * ( this->settings.Fsk.PreambleLen +
                                                             ( ( Read( REG_SYNCCONFIG ) &
                                                                ~RF_SYNCCONFIG_SYNCSIZE_MASK ) +
                                                             1.0 ) + 10.0 ) /
                                                            ( double )this->settings.Fsk.Datarate ) * 1e6 ) + 4000 );
            }
            else
            {
                this->settings.State = RF_IDLE;
                rxTimeoutSyncWord.detach( );
            }
        }
        if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->RxTimeout != NULL ) )
        {
            this->RadioEvents->RxTimeout( );
        }
        break;
    case RF_TX_RUNNING:
        this->settings.State = RF_IDLE;
        if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->TxTimeout != NULL ) )
        {
            this->RadioEvents->TxTimeout( );
        }
        break;
    default:
        break;
    }
}

void SX1272::OnDio0Irq( void )
{
    volatile uint8_t irqFlags = 0;

    switch( this->settings.State )
    {
        case RF_RX_RUNNING:
            //TimerStop( &RxTimeoutTimer );
            // RxDone interrupt
            switch( this->settings.Modem )
            {
            case MODEM_FSK:
                if( this->settings.Fsk.CrcOn == true )
                {
                    irqFlags = Read( REG_IRQFLAGS2 );
                    if( ( irqFlags & RF_IRQFLAGS2_CRCOK ) != RF_IRQFLAGS2_CRCOK )
                    {
                        // Clear Irqs
                        Write( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                                    RF_IRQFLAGS1_PREAMBLEDETECT |
                                                    RF_IRQFLAGS1_SYNCADDRESSMATCH );
                        Write( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

                        rxTimeoutTimer.detach( );

                        if( this->settings.Fsk.RxContinuous == false )
                        {
                            rxTimeoutSyncWord.detach( );
                            this->settings.State = RF_IDLE;
                        }
                        else
                        {
                            // Continuous mode restart Rx chain
                            Write( REG_RXCONFIG, Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                            rxTimeoutSyncWord.attach_us( this, &SX1272::OnTimeoutIrq, ceil( ( 8.0 * ( this->settings.Fsk.PreambleLen +
                                                                         ( ( Read( REG_SYNCCONFIG ) &
                                                                            ~RF_SYNCCONFIG_SYNCSIZE_MASK ) +
                                                                         1.0 ) + 10.0 ) /
                                                                        ( double )this->settings.Fsk.Datarate ) * 1e6 ) + 4000 );
                        }


                        if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->RxError != NULL ) )
                        {
                            this->RadioEvents->RxError( );
                        }
                        this->settings.FskPacketHandler.PreambleDetected = false;
                        this->settings.FskPacketHandler.SyncWordDetected = false;
                        this->settings.FskPacketHandler.NbBytes = 0;
                        this->settings.FskPacketHandler.Size = 0;
                        break;
                    }
                }

                // Read received packet size
                if( ( this->settings.FskPacketHandler.Size == 0 ) && ( this->settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( this->settings.Fsk.FixLen == false )
                    {
                        ReadFifo( ( uint8_t* )&this->settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        this->settings.FskPacketHandler.Size = Read( REG_PAYLOADLENGTH );
                    }
                    ReadFifo( rxtxBuffer + this->settings.FskPacketHandler.NbBytes, this->settings.FskPacketHandler.Size - this->settings.FskPacketHandler.NbBytes );
                    this->settings.FskPacketHandler.NbBytes += ( this->settings.FskPacketHandler.Size - this->settings.FskPacketHandler.NbBytes );
                }
                else
                {
                    ReadFifo( rxtxBuffer + this->settings.FskPacketHandler.NbBytes, this->settings.FskPacketHandler.Size - this->settings.FskPacketHandler.NbBytes );
                    this->settings.FskPacketHandler.NbBytes += ( this->settings.FskPacketHandler.Size - this->settings.FskPacketHandler.NbBytes );
                }

                rxTimeoutTimer.detach( );

                if( this->settings.Fsk.RxContinuous == false )
                {
                    this->settings.State = RF_IDLE;
                    rxTimeoutSyncWord.detach( );
                }
                else
                {
                    // Continuous mode restart Rx chain
                    Write( REG_RXCONFIG, Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                    rxTimeoutSyncWord.attach_us( this, &SX1272::OnTimeoutIrq, ceil( ( 8.0 * ( this->settings.Fsk.PreambleLen +
                                                                 ( ( Read( REG_SYNCCONFIG ) &
                                                                    ~RF_SYNCCONFIG_SYNCSIZE_MASK ) +
                                                                 1.0 ) + 10.0 ) /
                                                                ( double )this->settings.Fsk.Datarate ) * 1e6 ) + 4000 );
                }

                if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->RxDone != NULL ) )
                {
                    this->RadioEvents->RxDone( rxtxBuffer, this->settings.FskPacketHandler.Size, this->settings.FskPacketHandler.RssiValue, 0 );
                }
                this->settings.FskPacketHandler.PreambleDetected = false;
                this->settings.FskPacketHandler.SyncWordDetected = false;
                this->settings.FskPacketHandler.NbBytes = 0;
                this->settings.FskPacketHandler.Size = 0;
                break;
            case MODEM_LORA:
                {
                    int8_t snr = 0;

                    // Clear Irq
                    Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                    irqFlags = Read( REG_LR_IRQFLAGS );
                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                        if( this->settings.LoRa.RxContinuous == false )
                        {
                            this->settings.State = RF_IDLE;
                        }
                        rxTimeoutTimer.detach( );

                        if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->RxError != NULL ) )
                        {
                            this->RadioEvents->RxError( );
                        }
                        break;
                    }

                    this->settings.LoRaPacketHandler.SnrValue = Read( REG_LR_PKTSNRVALUE );
                    if( this->settings.LoRaPacketHandler.SnrValue & 0x80 ) // The SNR sign bit is 1
                    {
                        // Invert and divide by 4
                        snr = ( ( ~this->settings.LoRaPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                        snr = -snr;
                    }
                    else
                    {
                        // Divide by 4
                        snr = ( this->settings.LoRaPacketHandler.SnrValue & 0xFF ) >> 2;
                    }

                    int16_t rssi = Read( REG_LR_PKTRSSIVALUE );
                    if( snr < 0 )
                    {
                        this->settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET + rssi + ( rssi >> 4 ) +
                                                                      snr;
                    }
                    else
                    {
                        this->settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET + rssi + ( rssi >> 4 );
                    }

                    this->settings.LoRaPacketHandler.Size = Read( REG_LR_RXNBBYTES );
                    ReadFifo( rxtxBuffer, this->settings.LoRaPacketHandler.Size );

                    if( this->settings.LoRa.RxContinuous == false )
                    {
                        this->settings.State = RF_IDLE;
                    }
                    rxTimeoutTimer.detach( );

                    if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->RxDone != NULL ) )
                    {
                        this->RadioEvents->RxDone( rxtxBuffer, this->settings.LoRaPacketHandler.Size, this->settings.LoRaPacketHandler.RssiValue, this->settings.LoRaPacketHandler.SnrValue );
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            txTimeoutTimer.detach( );
            // TxDone interrupt
            switch( this->settings.Modem )
            {
            case MODEM_LORA:
                // Clear Irq
                Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
                // Intentional fall through
            case MODEM_FSK:
            default:
                this->settings.State = RF_IDLE;
                if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->TxDone != NULL ) )
                {
                    this->RadioEvents->TxDone( );
                }
                break;
            }
            break;
        default:
            break;
    }
}

void SX1272::OnDio1Irq( void )
{
    switch( this->settings.State )
    {
        case RF_RX_RUNNING:
            switch( this->settings.Modem )
            {
            case MODEM_FSK:
                // FifoLevel interrupt
                // Read received packet size
                if( ( this->settings.FskPacketHandler.Size == 0 ) && ( this->settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( this->settings.Fsk.FixLen == false )
                    {
                        ReadFifo( ( uint8_t* )&this->settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        this->settings.FskPacketHandler.Size = Read( REG_PAYLOADLENGTH );
                    }
                }

                if( ( this->settings.FskPacketHandler.Size - this->settings.FskPacketHandler.NbBytes ) > this->settings.FskPacketHandler.FifoThresh )
                {
                    ReadFifo( ( rxtxBuffer + this->settings.FskPacketHandler.NbBytes ), this->settings.FskPacketHandler.FifoThresh );
                    this->settings.FskPacketHandler.NbBytes += this->settings.FskPacketHandler.FifoThresh;
                }
                else
                {
                    ReadFifo( ( rxtxBuffer + this->settings.FskPacketHandler.NbBytes ), this->settings.FskPacketHandler.Size - this->settings.FskPacketHandler.NbBytes );
                    this->settings.FskPacketHandler.NbBytes += ( this->settings.FskPacketHandler.Size - this->settings.FskPacketHandler.NbBytes );
                }
                break;
            case MODEM_LORA:
                // Sync time out
                rxTimeoutTimer.detach( );
                this->settings.State = RF_IDLE;
                if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->RxTimeout != NULL ) )
                {
                    this->RadioEvents->RxTimeout( );
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( this->settings.Modem )
            {
            case MODEM_FSK:
                // FifoEmpty interrupt
                if( ( this->settings.FskPacketHandler.Size - this->settings.FskPacketHandler.NbBytes ) > this->settings.FskPacketHandler.ChunkSize )
                {
                    WriteFifo( ( rxtxBuffer + this->settings.FskPacketHandler.NbBytes ), this->settings.FskPacketHandler.ChunkSize );
                    this->settings.FskPacketHandler.NbBytes += this->settings.FskPacketHandler.ChunkSize;
                }
                else
                {
                    // Write the last chunk of data
                    WriteFifo( rxtxBuffer + this->settings.FskPacketHandler.NbBytes, this->settings.FskPacketHandler.Size - this->settings.FskPacketHandler.NbBytes );
                    this->settings.FskPacketHandler.NbBytes += this->settings.FskPacketHandler.Size - this->settings.FskPacketHandler.NbBytes;
                }
                break;
            case MODEM_LORA:
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

void SX1272::OnDio2Irq( void )
{
    switch( this->settings.State )
    {
        case RF_RX_RUNNING:
            switch( this->settings.Modem )
            {
            case MODEM_FSK:
                if( ( this->settings.FskPacketHandler.PreambleDetected == true ) && ( this->settings.FskPacketHandler.SyncWordDetected == false ) )
                {
                    rxTimeoutSyncWord.detach( );

                    this->settings.FskPacketHandler.SyncWordDetected = true;

                    this->settings.FskPacketHandler.RssiValue = -( Read( REG_RSSIVALUE ) >> 1 );

                    this->settings.FskPacketHandler.AfcValue = ( int32_t )( double )( ( ( uint16_t )Read( REG_AFCMSB ) << 8 ) |
                                                                           ( uint16_t )Read( REG_AFCLSB ) ) *
                                                                           ( double )FREQ_STEP;
                    this->settings.FskPacketHandler.RxGain = ( Read( REG_LNA ) >> 5 ) & 0x07;
                }
                break;
            case MODEM_LORA:
                if( this->settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        this->RadioEvents->FhssChangeChannel( ( Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( this->settings.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                if( this->settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        this->RadioEvents->FhssChangeChannel( ( Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

void SX1272::OnDio3Irq( void )
{
    switch( this->settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        if( ( Read( REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED )
        {
            // Clear Irq
            Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
            if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->CadDone != NULL ) )
            {
                this->RadioEvents->CadDone( true );
            }
        }
        else
        {
            // Clear Irq
            Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE );
            if( ( this->RadioEvents != NULL ) && ( this->RadioEvents->CadDone != NULL ) )
            {
                this->RadioEvents->CadDone( false );
            }
        }
        break;
    default:
        break;
    }
}

void SX1272::OnDio4Irq( void )
{
    switch( this->settings.Modem )
    {
    case MODEM_FSK:
        {
            if( this->settings.FskPacketHandler.PreambleDetected == false )
            {
                this->settings.FskPacketHandler.PreambleDetected = true;
            }
        }
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

void SX1272::OnDio5Irq( void )
{
    switch( this->settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}
