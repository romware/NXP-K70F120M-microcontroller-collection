/*! @file median.c
 *
 *  @brief Median filter.
 *
 *  This contains the functions for performing a median filter on byte-sized data.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
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

  if (n1 > n2)
  {
    if (n2 > n3)
    {
      median = n2; // n1 > n2 > n3
    }
    else if (n1 > n3)
    {
      median = n3; // n1 > n3 > n2
    }
    else
    {
      median = n1; // n3 > n1 > n2
    }
  }
  else
  {
    if (n3 > n2)
    {
      median = n2; // n3 > n2 > n1
    }
    else if (n1 > n3)
    {
      median = n1; // n2 > n1 > n3
    }
    else
    {
      median = n3; // n2 > n3 > n1
    }
  }
  return median;
}
/* END median */
/*!
** @}
*/
