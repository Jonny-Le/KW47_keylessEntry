#ifndef FUNCTION_LIB_H
#define FUNCTION_LIB_H

#include <stdint.h>
#include <stddef.h>

/* Mock: FLib_MemSet is provided by the test harness */
void FLib_MemSet(void *ptr, uint8_t val, size_t size);

#endif /* FUNCTION_LIB_H */
