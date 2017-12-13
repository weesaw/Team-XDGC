#ifndef _SECONDAPPROACH_H_
#define _SECONDAPPROACH_H_

#include <Arduino.h>

void clear_buffer(int *arr, float val, int n);
float calculate_range(float *arr, int n);
float calculate_mean_interval(float *arr, int start, int n);
void calculate_min_max_interval(float *arr, int start, int n, float* result);
float calculate_std_interval(float *arr, int start, int n);
int find_peaks_and_troughs(
		float *arr, 	// signal 
		int n_samples, 	// number of samples present in the signal
		// arrays that will store the indicies of the located
		// peaks and troughs
		int *P, int *T,
		// number of peaks (n_P) and number of troughs (n_T)
		// found in the data set *arr
		int *n_P, int *n_T
		);
void getAxisInfo(float *arr, int start, int length, float* result);
int extract_features(float *t, float *y2_gyro, float *x2_accel, float *y_gyro, float *x_accel, float *z_gyro, int N_SAMPLES, float* features);


#endif // _SECONDAPPROACH_H_