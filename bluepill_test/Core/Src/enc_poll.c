//#ifndef SRC_ENC_POLL_C_
//#define SRC_ENC_POLL_C_
//
//#include "enc_poll.h"
//
//uint32_t enc_curr[n_quad] = {0};
//uint32_t enc_prev[n_quad] = {0};
//
//extern TIM_HandleTypeDef htim2;
//
//TIM_HandleTypeDef* tim_arr[n_quad] = {&htim2};
//
//int16_t diff[n_quad] = {0};
//int32_t pos[n_quad] = {0}; //absolute position
//
////int16_t angPos[n_quad] = {0};
//
//void enc_poll_update(){
//	for (int i = 0; i<n_quad; i++){
//		enc_prev[i] = enc_curr[i];
//
//		enc_curr[i] = ((*(tim_arr[i])).Instance)->CNT; //(*htimx.Instance) = TIMx
//
//		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(tim_arr[i])){//should give negative diff
//			if (enc_curr[i] == enc_prev[i]){ //zero
//				diff[i] = 0;
//			} else if (enc_curr[i] < enc_prev[i]){ //normal
//				diff[i] = enc_curr[i] - enc_prev[i];
//			}
//			else { //(enc_curr[i] > enc_prev[i]){//appears to increase, so underflow
//				diff[i] = -(__HAL_TIM_GET_AUTORELOAD(tim_arr[i]) - enc_curr[i] + enc_prev[i]);
//			}
//		}
//		else{//should positive diff
//			if (enc_curr[i] == enc_prev[i]){ //zero
//				diff[i] = 0;
//			} else if (enc_curr[i] > enc_prev[i]){ //normal
//				diff[i] = enc_curr[i] - enc_prev[i];
//			}
//			else {//(enc_curr[i] < enc_prev[i]){//appears to increase, so underflow
//				diff[i] = __HAL_TIM_GET_AUTORELOAD(tim_arr[i]) + enc_curr[i] - enc_prev[i];
//			}
//		}
//
//		pos[i] += diff[i]; //absolute position
//	}
//}
//
//#endif /* SRC_ENC_POLL_C_ */
