/*! @file median.c
 *
 *  @brief Median filter.
 *
 *  This contains the functions for performing a median filter on byte-sized data.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-29
 */
/*!
**  @addtogroup median_module median module documentation
**  @{
*/
/* MODULE median */

// new types
#include "median.h"

/*! @brief Median filters 3 bytes.
 *
 *  @param n1 is the first  of 3 bytes for which the median is sought.
 *  @param n2 is the second of 3 bytes for which the median is sought.
 *  @param n3 is the third  of 3 bytes for which the median is sought.
 */
uint8_t Median_Filter3(const uint8_t n1, const uint8_t n2, const uint8_t n3)
{
  uint8_t median;
  uint8_t n1pos = n1 + 128;
  uint8_t n2pos = n2 + 128;
  uint8_t n3pos = n3 + 128;

  if (n1pos > n2pos)
  {
    if (n2pos > n3pos)
    {
      median = n2pos; // n1 > n2 > n3
    }
    else if (n1pos > n3pos)
    {
      median = n3pos; // n1 > n3 > n2
    }
    else
    {
      median = n1pos; // n3 > n1 > n2
    }
  }
  else
  {
    if (n3pos > n2pos)
    {
      median = n2pos; // n3 > n2 > n1
    }
    else if (n1pos > n3pos)
    {
      median = n1pos; // n2 > n1 > n3
    }
    else
    {
      median = n3pos; // n2 > n3 > n1
    }
  }
  return median - 128;
}
/* END median */
/*!
** @}
*/
