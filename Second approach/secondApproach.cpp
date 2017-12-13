#include "secondApproach.h"



void clear_buffer(int *arr, float val, int n) 
{
	int i;
	for (i = 0; i < n; i++) {
		arr[i] = val;
	}
}


float calculate_range(float *arr, int n)
{
	float min=0;
	float max=0;
	float range = 0;
	int i = 0;
	for (i=0;i<n;i++)
	{
	   if (i==0){
	   	min=arr[i];
	    max=arr[i];
	   }
	   else{
	    if (arr[i]<min) min=arr[i];
	    if (arr[i]>max) max=arr[i];
	   }

	}
	range = max - min;

	//printf("The max is %10.8lf \n", max);
	//printf("The min is %10.8lf \n", min);
	//printf("The range is %10.8lf \n", range);
	return range;

}

//Calculate mean in the interval
float calculate_mean_interval(float *arr, int start, int n)
{
	float total;
	int i;

	total = 0.0f;
	for (i = 0; i < n; i++) {
		total += arr[start+i];
	}

	return total/((float) n);
}



//Calculate min max in a interval
void calculate_min_max_interval(float *arr, int start, int n, float* result)
{
	float min=0;
	float max=0;
	//float range = 0;
	int i=0;
	for (i=0;i<n;i++)
	{
	   if (i==0){
	   	min=arr[start+i];
	    max=arr[start+i];
	   }
	   else{
	    if (arr[start+i]<min) min=arr[start+i];
	    if (arr[start+i]>max) max=arr[start+i];
	   }

	}
	result[0] = min;
	result[1] = max;
	return;
}


//Calculate std interval
float calculate_std_interval(float *arr, int start, int n)
{
	float mean = 0.0;
	float std = 0.0;
	int i=0;

    mean = calculate_mean_interval(arr, start, n);

    for(i=0; i<n; ++i)
        std += pow(arr[start+i] - mean, 2);

    return sqrt(std/n);
}




int 
find_peaks_and_troughs(
		float *arr, 	// signal 
		int n_samples, 	// number of samples present in the signal
		// arrays that will store the indicies of the located
		// peaks and troughs
		int *P, int *T,
		// number of peaks (n_P) and number of troughs (n_T)
		// found in the data set *arr
		int *n_P, int *n_T
		)
{
	int a, b, i, d, _n_P, _n_T;

	i = -1; d = 0; a = 0; b = 0;
	_n_P = 0; _n_T = 0;

	clear_buffer(P, 0.0f, n_samples);
	clear_buffer(T, 0.0f, n_samples);

	// overwirte relative threshold to a ratio of the range
	float E = 0.3 * calculate_range(arr, n_samples);

	while (i != n_samples) {
		i++;
		if (d == 0) {
			if (arr[a] >= (arr[i] + E)) {
				P[_n_P] = a;
				_n_P++;
				b = i;
				d = 2;
			} else if (arr[i] >= (arr[b] + E)) {
				T[_n_T] = b;
				_n_T++;
				a = i;
				d = 1;
			}
			if (arr[a] <= arr[i]) {
				a = i;
			} else if (arr[i] <= arr[b]) {
				b = i;
			}
		} else if (d == 1) {
			if (arr[a] <= arr[i]) {
				a = i;
			} else if (arr[a] >= (arr[i] + E)) {
				/*
				 * Peak has been detected.
				 * Add index at detected peak
				 * to array of peak indicies
				 * increment count of peak indicies
				 */
				P[_n_P] = a;
				_n_P++;
				b = i;
				d = 2;
			}
		} else if (d == 2) {
			if (arr[i] <= arr[b]) {
				b = i;
			} else if (arr[i] >= (arr[b] + E)) {
				/*
				 * Trough has been detected.
				 * Add index at detected trough
				 * to array of trough indicies
				 * increment count of trough indicies
				 */
				T[_n_T] = b;
				_n_T++;
				a = i;
				d = 1;
			}
		}
	}

	(*n_P) = _n_P;
	(*n_T) = _n_T;
	return 0;
}



void getAxisInfo(float *arr, int start, int length, float* result)
{
 	float min_max[2];
 	calculate_min_max_interval(arr, start, length, min_max);
 	result[0] = min_max[0];
 	result[1] = min_max[1];
    result[2] =  calculate_mean_interval(arr, start, length);
    result[3] =  calculate_std_interval(arr, start, length);
    //free(segment1);
    return;
}



int extract_features(float *t, float *y2_gyro, float *x2_accel, float *y_gyro, float *x_accel, float *z_gyro, int N_SAMPLES, float* features){
    
    int n_P; 	// number of peaks
    int n_T; 	// number of troughs
    //int n_S; 	// number of strides
    int i, idx;
    int P_i[N_SAMPLES], T_i[N_SAMPLES];//, S_i[N_SAMPLES]; 
    
    float center = 0;
    //float pk_threshold = 0;
    
    int rv = find_peaks_and_troughs(
                                    y2_gyro,
                                    N_SAMPLES,
                                    P_i, T_i,
                                    &n_P, &n_T);
    
    if(n_P<1 || n_T<2)
    	return 0;

    int k, idx_next;
    float stride_time = 0.2;
    int n_P_new = n_P;
    int n_T_new = n_T;
    
    // Process Peaks
    
    for (i = 0; i < n_P_new-1; i++)
    {
        idx = P_i[i];
        idx_next = P_i[i+1];
        if (t[idx_next] - t[idx] < stride_time) // Strides need to be at least 0.5 second apart
        {
            // Eliminate the second peak by shifting remaining elements to the left
            for (k = 0; k < n_P_new-i-2; k++)
            {
                P_i[i+k+1] = P_i[i+k+2];
            }
            
            i--; // Still need to look at the current peak in next iteration
            n_P_new--; // One less peak now
            
        }
    }
    //printf("%d peaks removed\n", n_P - n_P_new);

    if (n_P_new>2)
    	return 0;

    n_P = n_P_new;
    
    // Process Troughs
    for (i = 0; i < n_T_new-1; i++)
    {
        idx = T_i[i];
        idx_next = T_i[i+1];
        if (t[idx_next] - t[idx] < stride_time) // Strides need to be at least 0.5 second apart
        {
            // Eliminate the second peak by shifting remaining elements to the left
            for (k = 0; k < n_T_new-i-2; k++)
            {
                T_i[i+k+1] = T_i[i+k+2];
            }
            
            i--; // still need to look at the current trough in next iteration
            n_T_new--; // One less trough now
            
        }
    }
    
    
    // add one more condition
    // delete peaks or troughs that are too close to offset value
    if (n_T_new>2)
    	return 0;
    
    //printf("%d troughs removed\n", n_T - n_T_new);
    n_T = n_T_new;
    

    float peak_value, trough_value;
    int peak_idx, T1_idx = T_i[0], T2_idx = T_i[1];

    if(y2_gyro[T1_idx] > y2_gyro[T2_idx])
    	trough_value = y2_gyro[T2_idx];
    else
    	trough_value = y2_gyro[T1_idx];

    for(i=0; i<n_P; i++)
    {
    	if(P_i[i] > T_i[0])
    	{
    		peak_value = y2_gyro[P_i[i]];
    		peak_idx = P_i[i];
    		break;
    	}
    }
    
    float period = t[T2_idx] - t[T1_idx];
    int segment_length = T2_idx - T1_idx + 1;
    float gy2_result[4], ax2_result[4] , gy1_result[4], ax1_result[4], gz1_result[4];

    getAxisInfo(y2_gyro, T1_idx, segment_length, gy2_result);
    getAxisInfo(x2_accel, T1_idx, segment_length, ax2_result);
    getAxisInfo(y_gyro, T1_idx, segment_length, gy1_result);
    getAxisInfo(x_accel, T1_idx, segment_length, ax1_result);
    getAxisInfo(z_gyro, T1_idx, segment_length, gz1_result);
    features[0] = trough_value;
    features[1] = peak_value;
    features[2] = period;
    for(k=0; k<4;k++) features[k+3] = gy2_result[k];
    for(k=0; k<4;k++) features[k+7] = ax1_result[k];
    for(k=0; k<4;k++) features[k+11] = ax2_result[k];
    for(k=0; k<4;k++) features[k+15] =  gy1_result[k];
    for(k=0; k<4;k++) features[k+19] =  gz1_result[k];

	return n_P;
}

