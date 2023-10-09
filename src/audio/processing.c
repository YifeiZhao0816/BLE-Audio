#include "processing.h"
#include "arm_math.h"

#define STEREO_BLOCK_BYTES 192
#define MONO_BLOCK_BYTES (STEREO_BLOCK_BYTES/2)
#define BYTE_DEPTH 2
#define MONO_BLOCK_SAMPLES (MONO_BLOCK_BYTES/BYTE_DEPTH)

#define COEFFICIENT 24

static arm_lms_instance_q15 lms_instance;
static q15_t State[MONO_BLOCK_SAMPLES + COEFFICIENT - 1];
static q15_t errorArr[MONO_BLOCK_SAMPLES];
static q15_t coeff_p[COEFFICIENT];

static sync_buf RX_buf;

void InitBuffer() {
    memset(RX_buf.content, 0, STEREO_BLOCK_BYTES*Blks_Per_Frame);
    RX_buf.blk_count = 0;
}

void pushBuffer(int8_t* block_ptr) {
    memcpy(RX_buf.content, &RX_buf.content[STEREO_BLOCK_BYTES], STEREO_BLOCK_BYTES*9);
    memcpy(&RX_buf.content[STEREO_BLOCK_BYTES*9], block_ptr, STEREO_BLOCK_BYTES);
    RX_buf.blk_count++;
}


//Initilize a FIR filter used for LMS filtering
void InitFIRFilter(){
    float32_t learningrate_float = 0.0002;
    q15_t learningRate;
    arm_float_to_q15(&learningrate_float, &learningRate, 1);
    arm_lms_init_q15(&lms_instance, COEFFICIENT, coeff_p,  State, learningRate, MONO_BLOCK_SAMPLES, 0);
}

void filterBLK(int16_t* input, int16_t* reference, int16_t* output){
    // Allocate q15_t buffers
    // update 9/26/2023: change all to float32_t
    q15_t input_mono[MONO_BLOCK_SAMPLES]; 
    q15_t reference_mono[MONO_BLOCK_SAMPLES];
    q15_t output_mono[MONO_BLOCK_SAMPLES];
    
    // input has one channel, so we take the last 48 samples
    // reference has two channels, with samples in each channel stored alternatively, so we take
    // 48 samples from the left channel
    int i;
    for (i = 0; i < MONO_BLOCK_SAMPLES; i++){
        input_mono[i] = (q15_t) input[2*i];
        reference_mono[i] = (q15_t) reference[2*i];
    }

    // arm_lms_q15(&lms_instance, input_mono, input_mono, output_mono, errorArr, MONO_BLOCK_SAMPLES);

    for (i = 0; i < MONO_BLOCK_SAMPLES; i++){
        output[2*i] = (int16_t)input_mono[i];
        output[2*i + 1] = (int16_t)input_mono[i];
    }
}

void filterFIR(int16_t* reference, int16_t* output){
    if (RX_buf.blk_count >= 10){
        int blk;
        for (blk = 0; blk < 10; blk++){
            filterBLK(&RX_buf.content[blk*STEREO_BLOCK_BYTES], &reference[blk*STEREO_BLOCK_BYTES], output);
        }
        RX_buf.blk_count = 0;
    }
}

// Functions for initial testing and debugging
void* getCoeffPtr(){
    return coeff_p;
}

void* getErrPtr(){
    return errorArr;
}

void TestLibrary(uint8_t * decoded_input, uint8_t * processed_decoded_output, size_t decoded_data_length){
    //convert PCM to float points
    int N = 1;
    float impulse[N];
    int n;
    for (n = 0; n < N; n++) {
        if(n == 0) {
            impulse[n] = 1.0;
        } else {
            impulse[n] = 0.0;
        }
    }
    q7_t newimpulse[N];
    arm_float_to_q7(impulse, newimpulse, N);
    arm_conv_q7((q7_t *)decoded_input, decoded_data_length, newimpulse, N, processed_decoded_output);
}