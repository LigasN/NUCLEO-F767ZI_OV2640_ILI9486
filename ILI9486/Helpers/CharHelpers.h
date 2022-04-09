/*****************************************************************************
 * | File      	:	CharHelpers.h
 * | Author      :  Norbert Ligas
 * | Function    :	Some methods helping with operations on char strings
 *----------------
 * | Date        :   2022-03-09
 *
 ******************************************************************************/

#ifndef CHAR_HELPERS_H
#define CHAR_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>

/******************************************************************************
 function:	Shifts chars in the array of specified size (secure way)
            'count' times to the right.
 parameter: str		 :   string to operate on
            count	 :	amount of steps to shift
            str_size :   size of the string
 ******************************************************************************/
void shiftRightCharsInArray(char* str, size_t count, size_t str_size)
{
    size_t size = strnlen(str, str_size);

    for (size_t i = 1U; i <= size; ++i)
    {
        if (size + count - i < str_size)
            str[size + count - i] = str[size - i];
    }
}

#endif // CHAR_HELPERS_H
