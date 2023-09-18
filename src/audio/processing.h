#ifndef _PROCESSING_H_
#define _PROCESSING_H_

#include "arm_math.h"

void TestLibrary(uint8_t * decoded_input, uint8_t * processed_decoded_output, size_t decoded_data_length);
void InitFIRFilter();
void filterFIR(int16_t * input, int16_t* reference, int16_t* output);
void* getCoeffPtr();
void* getErrPtr();

#endif
