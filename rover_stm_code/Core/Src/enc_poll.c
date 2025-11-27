#include "enc_poll.h"
#include "main.h"
//normal timer encoders
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;
extern TIM_HandleTypeDef  htim4;
extern TIM_HandleTypeDef  htim6;
extern TIM_HandleTypeDef  htim7;
extern TIM_HandleTypeDef  htim8;
extern TIM_HandleTypeDef  htim16;

extern uint8_t TxData_buf[];

uint32_t enc_curr[NUM_QUAD] = {0};
uint32_t enc_prev[NUM_QUAD] = {0};

int16_t diff[NUM_QUAD] = {0};
int16_t diff2[NUM_QUAD] = {0};
int16_t diff_prev[NUM_QUAD] = {0};

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
int32_t dat[3*NUM_QUAD];

void timer_quad_poll(){
	enc_prev[i] = enc_curr[i];
	diff_prev[i] = diff[i];

	enc_curr[i] = __HAL_TIM_GET_COUNTER(&htim2);
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){//should give negative diff
		if (enc_curr[i] == enc_prev[i]){ //zero
			diff[i] = 0;
		} else if (enc_curr[i] < enc_prev[i]){ //normal
			diff[i] = enc_curr[i] - enc_prev[i];
		}
		else { //(enc_curr[i] > enc_prev[i]){//appears to increase, so underflow
			diff[i] = -(__HAL_TIM_GET_AUTORELOAD(&htim2) - enc_curr[i] + enc_prev[i]);
		}
	}
	else{//should positive diff
		if (enc_curr[i] == enc_prev[i]){ //zero
			diff[i] = 0;
		} else if (enc_curr[i] > enc_prev[i]){ //normal
			diff[i] = enc_curr[i] - enc_prev[i];
		}
		else {//(enc_curr[i] < enc_prev[i]){//appears to increase, so underflow
			diff[i] = __HAL_TIM_GET_AUTORELOAD(&htim2) + enc_curr[i] - enc_prev[i];
		}
	}

	pos[i] += diff[i]; //absolute position
	diff2[i] = diff[i] - diff_prev[i];
}

void timer_update_TX(){
	for (int i = 0; i < NUM_QUAD; i++){
		dat[i] = pos[i];
		dat[i + NUM_QUAD] = diff[i];
		dat[i + 2*NUM_QUAD] = diff2[i];

		TxData_buf[4*i] = dat[i]&0x000000FF;
		TxData_buf[4*i+1] = (dat[i]&0x0000FF00)>>8;
		TxData_buf[4*i+2] = (dat[i]&0x00FF0000)>>16;
		TxData_buf[4*i+3] = (dat[i]&0xFF000000)>>24;

		TxData_buf[4*(i+NUM_QUAD)] = dat[i+NUM_QUAD]&0x000000FF;
		TxData_buf[4*(i+NUM_QUAD)+1] = (dat[i+NUM_QUAD]&0x0000FF00)>>8;
		TxData_buf[4*(i+NUM_QUAD)+2] = (dat[i+NUM_QUAD]&0x00FF0000)>>16;
		TxData_buf[4*(i+NUM_QUAD)+3] = (dat[i+NUM_QUAD]&0xFF000000)>>24;

		TxData_buf[4*(i+2*NUM_QUAD)] = dat[i+2*NUM_QUAD]&0x000000FF;
		TxData_buf[4*(i+2*NUM_QUAD)+1] = (dat[i+2*NUM_QUAD]&0x0000FF00)>>8;
		TxData_buf[4*(i+2*NUM_QUAD)+2] = (dat[i+2*NUM_QUAD]&0x00FF0000)>>16;
		TxData_buf[4*(i+2*NUM_QUAD)+3] = (dat[i+2*NUM_QUAD]&0xFF000000)>>24;
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
}

void drill_update_TX(){
	int i = 0;
	gpio_pos += gpio_diff; //absolute position
	gpio_diff2 = gpio_diff - gpio_diff_prev;

	dat[i] = gpio_pos;
	dat[i + NUM_QUAD] = gpio_diff;
	dat[i + 2*NUM_QUAD] = gpio_diff2;

	TxData_buf[4*i+12] = dat[i]&0x000000FF;
	TxData_buf[4*i+13] = (dat[i]&0x0000FF00)>>8;
	TxData_buf[4*i+14] = (dat[i]&0x00FF0000)>>16;
	TxData_buf[4*i+15] = (dat[i]&0xFF000000)>>24;

	TxData_buf[4*(i+NUM_QUAD)+12] = dat[i+NUM_QUAD]&0x000000FF;
	TxData_buf[4*(i+NUM_QUAD)+13] = (dat[i+NUM_QUAD]&0x0000FF00)>>8;
	TxData_buf[4*(i+NUM_QUAD)+14] = (dat[i+NUM_QUAD]&0x00FF0000)>>16;
	TxData_buf[4*(i+NUM_QUAD)+15] = (dat[i+NUM_QUAD]&0xFF000000)>>24;

	TxData_buf[4*(i+2*NUM_QUAD)+12] = dat[i+2*NUM_QUAD]&0x000000FF;
	TxData_buf[4*(i+2*NUM_QUAD)+13] = (dat[i+2*NUM_QUAD]&0x0000FF00)>>8;
	TxData_buf[4*(i+2*NUM_QUAD)+14] = (dat[i+2*NUM_QUAD]&0x00FF0000)>>16;
	TxData_buf[4*(i+2*NUM_QUAD)+15] = (dat[i+2*NUM_QUAD]&0xFF000000)>>24;
}
