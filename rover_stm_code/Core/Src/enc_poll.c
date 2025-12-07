#include "enc_poll.h"
#include "main.h"
//normal timer encoders
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
//extern TIM_HandleTypeDef  htim6;
//extern TIM_HandleTypeDef  htim7;
extern TIM_HandleTypeDef  htim8;
//extern TIM_HandleTypeDef  htim16;

TIM_HandleTypeDef* tim_arr[NUM_QUAD] = {&htim1, &htim2, &htim3, &htim4, &htim8};
//tim_arr[0] = htim1;
//tim_arr[1] = htim2;
//tim_arr[2] = htim3;
//tim_arr[3] = htim4;
//tim_arr[4] = htim8;

extern uint8_t TxData_buf[];

uint32_t enc_curr[NUM_QUAD] = {0};
uint32_t enc_prev[NUM_QUAD] = {0};

int16_t diff[NUM_QUAD] = {0};
int16_t diff2[NUM_QUAD] = {0};
int16_t diff_prev[NUM_QUAD] = {0};
int16_t MAX_THINGY[NUM_QUAD];

//drill encoder
extern volatile int32_t quad_count;
extern volatile uint8_t gpio_quad_counting_down;

int16_t gpio_diff = 0;
int16_t gpio_diff2 = 0;
int16_t gpio_diff_prev = 0;

int32_t gpio_pos = 0;
int32_t gpio_enc_curr = 0; //absolute position
int32_t gpio_enc_prev = 0;


int32_t pos[NUM_QUAD] = {0}; //absolute position

void timer_quad_poll(){
	for (int i = 0; i < NUM_QUAD; i++){
		enc_prev[i] = enc_curr[i];
		diff_prev[i] = diff[i];

		enc_curr[i] = (__HAL_TIM_GET_COUNTER(tim_arr[i]));

		diff[i] = enc_curr[i] - enc_prev[i];

		MAX_THINGY[i] = __HAL_TIM_GET_AUTORELOAD(&htim2);

		if (diff[i] > MAX_THINGY[i] / 2) {
			diff[i] -= MAX_THINGY[i];
		}

		if (diff[i] < -MAX_THINGY[i] / 2) {
			diff[i] += MAX_THINGY[i];
		}

		pos[i] += diff[i]; //absolute position
		diff2[i] = diff[i] - diff_prev[i];
	}
}

void timer_update_TX(){
	for (int i = 0; i < NUM_QUAD; i++){
		TxData_buf[12*i] = (pos[i]&0xFF000000)>>24;
		TxData_buf[12*i+1] = (pos[i]&0x00FF0000)>>16;
		TxData_buf[12*i+2] = (pos[i]&0x0000FF00)>>8;
		TxData_buf[12*i+3] = (pos[i]&0x000000FF);

		TxData_buf[12*i+4] = (diff[i]&0xFF000000)>>24;
		TxData_buf[12*i+5] = (diff[i]&0x00FF0000)>>16;
		TxData_buf[12*i+6] = (diff[i]&0x0000FF00)>>8;
		TxData_buf[12*i+7] = (diff[i]&0x000000FF);

		TxData_buf[12*i+8] = (diff2[i]&0xFF000000)>>24;
		TxData_buf[12*i+9] = (diff2[i]&0x00FF0000)>>16;
		TxData_buf[12*i+10] = (diff2[i]&0x0000FF00)>>8;
		TxData_buf[12*i+11] = (diff2[i]&0x000000FF);
	}
}

void drill_quad_poll(){
	gpio_enc_prev = gpio_enc_curr;
	gpio_diff_prev = gpio_diff;

	gpio_enc_curr = quad_count;
	if (gpio_quad_counting_down){//should give negative diff
		if (gpio_enc_curr == gpio_enc_prev){ //zero
			gpio_diff = 0;
		} else if (gpio_enc_curr < gpio_enc_prev){ //normal
			gpio_diff = gpio_enc_curr - gpio_enc_prev;
		}
		else { //(enc_curr[i] > enc_prev[i]){//appears to increase, so underflow
			gpio_diff = -(quad_count - gpio_enc_curr + gpio_enc_prev);
		}
	}
	else{//should positive diff
		if (gpio_enc_curr == gpio_enc_prev){ //zero
			gpio_diff = 0;
		} else if (gpio_enc_curr > gpio_enc_prev){ //normal
			gpio_diff = gpio_enc_curr - gpio_enc_prev;
		}
		else {//(enc_curr[i] < enc_prev[i]){//appears to increase, so underflow
			gpio_diff = quad_count + gpio_enc_curr - gpio_enc_prev;
		}
	}
	gpio_diff = gpio_diff;
	//gpio_pos += gpio_diff; //absolute position
	gpio_pos = gpio_enc_curr;
	gpio_diff2 = gpio_diff - gpio_diff_prev;
}

void drill_update_TX(){
	TxData_buf[12*NUM_QUAD] = (gpio_pos&0xFF000000)>>24;
	TxData_buf[12*NUM_QUAD+1] = (gpio_pos&0x00FF0000)>>16;
	TxData_buf[12*NUM_QUAD+2] = (gpio_pos&0x0000FF00)>>8;
	TxData_buf[12*NUM_QUAD+3] = (gpio_pos&0x000000FF);

	TxData_buf[12*NUM_QUAD+4] = (gpio_diff&0xFF000000)>>24;
	TxData_buf[12*NUM_QUAD+5] = (gpio_diff&0x00FF0000)>>16;
	TxData_buf[12*NUM_QUAD+6] = (gpio_diff&0x0000FF00)>>8;
	TxData_buf[12*NUM_QUAD+7] = (gpio_diff&0x000000FF);

	TxData_buf[12*NUM_QUAD+8] = (gpio_diff2&0xFF000000)>>24;
	TxData_buf[12*NUM_QUAD+9] = (gpio_diff2&0x00FF0000)>>16;
	TxData_buf[12*NUM_QUAD+10] = (gpio_diff2&0x0000FF00)>>8;
	TxData_buf[12*NUM_QUAD+11] = (gpio_diff2&0x000000FF);
}
