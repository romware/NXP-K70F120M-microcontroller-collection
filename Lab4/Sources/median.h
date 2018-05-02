/*! @file median.h
 *
 *  @brief Median filter.
 *
 *  This contains the functions for performing a median filter on byte-sized data.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-29
 */

#ifndef MEDIAN_H
#define MEDIAN_H

// New types
#include "types.h"

/*! @brief Median filters 3 bytes.
 *
 *  @param n1 is the first  of 3 bytes for which the median is sought.
 *  @param n2 is the second of 3 bytes for which the median is sought.
 *  @param n3 is the third  of 3 bytes for which the median is sought.
 */
uint8_t Median_Filter3(const uint8_t n1, const uint8_t n2, const uint8_t n3);

#endif
