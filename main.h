/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    ( C )2014 Semtech

Description: Contains the callbacks for the IRQs and any application related details

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
Last update: Antoine Boisadam
*/
#ifndef __MAIN_H__
#define __MAIN_H__

/*
 * Callback functions prototypes
 */

/*!
 * @brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * @brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * @brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * @brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * @brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*!
 * @brief Function which convert a unint8 to float
 */
float u8_to_float(uint8_t x, bool isTemp);

/*!
 * @brief function that print payload data
 */
void retrieve_data(uint8_t * payload);

/*!
 * @brief function that convert an encoded measurement into is real value
 */
void convert(uint8_t m, uint8_t t);

#endif // __MAIN_H__