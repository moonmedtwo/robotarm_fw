#ifndef UTIL_H__
#define UTIL_H__
/*!
 Break and buffer a float value - LSB first
 Public function defined in util.h
 */
float Util_buildFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);
/*!
 Build a float from a uint8_t array
 Public function defined in util.h
 */
float Util_parseFloat(uint8_t *pArray);
/*
 * @brief: Break and buffer a float value - LSB first
 * @return: pointer to last buffer
 */
uint8_t * Util_bufferFloat(uint8_t *pBuf, float val);
/*!
 Build a uint32_t out of 4 uint8_t variables
 Public function defined in util.h
 */
uint32_t Util_buildUint32(uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3);
/*!
 Build a uint32_t from a uint8_t array

 Public function defined in util.h
 */
uint32_t Util_parseUint32(uint8_t *pArray);
/*!
 Pull 1 uint8_t out of a uint32_t
 Public function defined in util.h
 */
uint8_t Util_breakUint32(uint32_t var, int byteNum);
/*!
 Break and buffer a uint32_t value - LSB first
 Public function defined in util.h
 */ 
uint8_t * Util_bufferUint32(uint8_t *pBuf, uint32_t val);

/*
 * @brief: call at the start of the program to get clock frequency
 */
void DelayInit(void);
/*
 * @brief: delay in X microseconds
 */
void DelayUs(uint32_t Micros);
/*
 * @brief: delay in X milliseconds
 */
void DelayMs(uint32_t Millis);

#endif //UTIL_H__
