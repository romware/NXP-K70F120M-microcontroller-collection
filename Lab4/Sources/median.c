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

/*! @brief Get sorted numbers.
 *
 *  @param numbers is the array of numbers to search through.
 *  @param length is the array length.
 */
static void SortNumbers(uint8_t numbers[], const uint8_t length)
{
  for (uint8_t i = 0; i < length-1; i++)
  {
    for (uint8_t j = 0; j < length-i-1; j++)
    {
      if (numbers[j] > numbers[j+1])
      {
        uint8_t tempNumber = numbers[j];
        numbers[j] = numbers[j+1];
        numbers[j+1] = tempNumber;
      }
    }
  }
}

/*! @brief Median filters 3 bytes.
 *
 *  @param n1 is the first  of 3 bytes for which the median is sought.
 *  @param n2 is the second of 3 bytes for which the median is sought.
 *  @param n3 is the third  of 3 bytes for which the median is sought.
 */
uint8_t Median_Filter3(const uint8_t n1, const uint8_t n2, const uint8_t n3)
{
  uint8_t numbers[3] = {n1,n2,n3};
  SortNumbers(numbers,3);
  return numbers[1];
}
/* END median */
/*!
** @}
*/
