/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "printk.h"
//#include "print_services.h"
#include "utils.h"

#define DEBUG
#if defined(DEBUG)

typedef unsigned char bool;
#define false 0
#define true 1

int __vsnprintf(char *dest, int size, const char *fmt, va_list args)
{
	int might_format = 0;
	int len = 0;
	char lastchar = 0;
	bool binary_format = false;

	if (!dest || !size)
		return 0;

	while (*fmt && len < size) {
		if (!might_format) {
			if (*fmt == '\n' && lastchar != '\r') {
				if (len < size) {
					lastchar = *dest++ = '\r', len++;
					continue;
				}
				else
					break;
			}
			else if (*fmt != '%') {
				if (len < size)
					lastchar = *dest++ = *fmt, len++;
				else
					break;
			} else
				might_format = 1;
		} else {
			if (*fmt == '%') {
				if (len < size)
					*dest++ = '%', len++;
				else
					break;
				might_format = 0;
			} else {
				switch (*fmt) {
				case '0':
				case '1':
					might_format |= 2;
					goto still_format;
					break;
				case '2':
					might_format |= 4;
					goto still_format;
					break;
				case '4':
					might_format |= 8;
					goto still_format;
					break;
				case 'b':
					binary_format = true;
					goto still_format;
					break;
				case 'd':
				case 'i':
				case 'u':
					if (!binary_format) {
						unsigned long num =
						    va_arg(args, unsigned long);
						unsigned long pos = 999999999;
						unsigned long remainder = num;
						int found_largest_digit = 0;

						if (*fmt != 'u'
						    && (num & (1 << 31))) {
							if (len < size)
								*dest++ =
								    '-', len++;
							num = (~num) + 1;
							remainder = num;
						}
						while (pos >= 9) {
							if (found_largest_digit
							    || remainder >
							    pos) {
								found_largest_digit
								    = 1;
								if (len < size)
									*dest++
									    =
									    (char)
									    ((remainder / (pos + 1)) + 48), len++;
								else
									break;
							}
							remainder %= (pos + 1);
							pos /= 10;
						}
						if (len < size)
							*dest++ =
							    (char)(remainder +
								   48), len++;
						break;
					}
				case 'x':
				case 'X':
				case 'p':{
						unsigned long num =
						    va_arg(args, unsigned long);
						int sz = sizeof(num) * 2;

						if (might_format & 8){
							sz = 4;
						}
						else if (might_format & 4){
							sz = 2;
						}
						else if (might_format & 2){
							sz = 1;
						}
						for (; sz; sz--) {
							char nibble;
							if (!binary_format) {
								nibble =
								    (num >>
								     ((sz -
								       1) << 2) & 0xf);
								nibble +=
								    nibble >
								    9 ? 87 : 48;

							} else {
								nibble =
								    (num >>
								     ((sz -
								       1) << 3) & 0xff);
							}
							if (len < size)
								*dest++ =
								    nibble,
								    len++;
							else
								break;
						}
						break;
					}
				case 's':{
						char *s = va_arg(args, char *);
						while (*s)
							if (len < size)
								*dest++ =
								    *s++, len++;
							else
								break;
						break;
					}
				case 'c':{
						char c = va_arg(args, int);
						if (len < size)
							*dest++ = c, len++;
						break;
					}
				default:
					if (len < size)
						*dest++ = '%', len++;
					if (len < size)
						*dest++ = *fmt, len++;
					break;
				}
				might_format = 0;
				still_format:
				(void)might_format;
			}
		}
		++fmt;
	}
	*dest = '\0';
	return len;
}

#define BUFFER_SIZE 1024
char print_buffer[BUFFER_SIZE];
char *current = print_buffer;

void add_to_print_buffer(char* buffer, int len) {
	int i;
	for (i = 0; i<len; i++) {
		current[0] = buffer[i];
		current++;
		if(current == (print_buffer + BUFFER_SIZE)) {
			current = print_buffer;
		}
	}
}

int log_level = LEVEL_PROFILE;
int printk(int level, const char *fmt, ...)
{
	int len = 0;

	if ((log_level > 0) && (level <= log_level))  {
		va_list args;
		char tmp[PRINTK_BUFSIZ];

		va_start(args, fmt);
		len = __vsnprintf(tmp, PRINTK_BUFSIZ, fmt, args);
		add_to_print_buffer(tmp, len);
		va_end(args);
	}

	return len;
}

#endif /* DEBUG */
