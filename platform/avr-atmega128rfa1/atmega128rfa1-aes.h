/**
 * \file
 *         Interface to the ATmega128rfa1 AES encryption/decryption functions
 * \author
 *         Stefan Lorenz <s69401@informatik.htw-dresden.de>
 */
#ifndef ATMEGA128RFA1_AES_H
#define ATMEGA128RFA1_AES_H

#include <stdint.h>
/**
 * \brief      Setup an AES key
 * \param key  A pointer to a 16-byte AES key
 *
 *             Write encryption or decryption key to KEY
 * 	       buffer(16 consecutive byte writes to AES_KEY)
 *
 *             ATmega128rfa1 only supports AES-128 standard(128 Bit/16 Byte key-length)
 *
 */
void atmega128rfa1_aes_set_key(uint8_t *key);
/**
 * \brief      Setup the AES mode
 * \param index The mode index: either 0 or 1.
 *
 *             This function sets up the AES mode.
 *
 *             0 sets up the AES-ECB-Mode.
 *	       1 sets up the AES-CBC-Mode.
 *
 *             Note: ECB decryption is not required for IEEE 802.15.4 or ZigBee security processing.
 *             The radio transceiver provides this functionality as an additional feature.
 *
 */
void atmega128rfa1_aes_set_mode(int index);
/**
 * \brief      Encrypt/decrypt data with AES
 * \param data A pointer to the data to be encrypted/decrypted
 * \param len  The length of the data to be encrypted/decrypted
 * \param direction The direction to edit the cipher text: either 0 = encrypt or 1 = decrypt
 *
 *             This function encrypts/decrypts data with AES. A
 *             pointer to the data is passed as a parameter, and the
 *             function overwrites the data with the encrypted/decrypted data.
 *
 */
void atmega128rfa1_aes_cipher(uint8_t *data, int len, int direction);

#endif /* ATMEGA128RFA1_AES_H */
