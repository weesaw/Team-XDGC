/* for file and terminal I/O */
#include <stdio.h>
/* for string manip */
#include <string.h>
/* for exit() */
#include <stdlib.h>
/* for fabsf() */
#include <math.h>

#include "functions.h"

#define BUFF_SIZE 1024

/*
 * sets first <n> values in <*arr> to <val>
 */
void clear_buffer(double *arr, double val, int n) 
{
	int i;
	for (i = 0; i < n; i++) {
		arr[i] = val;
	}
}

/*
 * Caculates mean of first <n> samples in <*arr>
 */
double calculate_mean(double *arr, int n)
{
	double total;
	int i;

	total = 0.0f;
	for (i = 0; i < n; i++) {
		total += arr[i];
	}

	return total/((double) n);
}

double calculate_min(double *arr, int n)
{
	double min=0;
	double max=0;
	double range = 0;
	int i=0;
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
	return min;
}

double calculate_max(double *arr, int n)
{
	double min=0;
	double max=0;
	double range = 0;
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
	return max;
}

double calculate_range(double *arr, int n)
{
	double min=0;
	double max=0;
	double range = 0;
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

double calculate_mean_righthalf(double *arr, int n)
{
	double total;
	int i;

	total = 0.0f;
	for (i = 0; i < n; i++) {
		if (i >= n/2)
		total += arr[i];
	}

	return 2 * total/((double) n);
}

double calculate_dc_offset(double *arr, int n)
{
	double RawData;
	double SmoothData;
	double *filered_signal;
	double LPF_Beta = 0.001; // 0<ß<1
							 // ß -> 0 smoother
							 // 0.001 for calculating de offset
	int i = 0;

	filered_signal = (double *) malloc(sizeof(double) * n);

    // LPF: Y(n) = (1-ß)*Y(n-1) + (ß*X(n))) = Y(n-1) - (ß*(Y(n-1)-X(n)));
    while(i <= n){
       // Function that brings Fresh Data into RawData
       RawData = arr[i];
       SmoothData = SmoothData - (LPF_Beta * (SmoothData - RawData));
       filered_signal[i] = SmoothData;
       i++;
    }

    // write filtered data to file
    char *file_name = "testLPF0001x.csv";
    FILE *fpt;

    printf("Attempting to write to file \'%s\'.\n", file_name);
		fpt = fopen(file_name, "w");
		if (fpt == NULL) {
			fprintf(stderr, 
					"Failed to write to file \'%s\'.\n", 
					file_name
				   );
			exit(EXIT_FAILURE);
		}

	fprintf(fpt, "S_i,S_t,S_x\n");
		for (i = 0; i < n; i++) {
			fprintf(fpt, "%20.10lf\n",
					filered_signal[i]
				   );
		}
		fprintf(fpt, "\n\n\n");

		fclose(fpt);

	// calculate offset
    return calculate_mean_righthalf(filered_signal, n);
}



double calculate_std(double *arr, int n)
{
	double mean = 0.0;
	double std = 0.0;
	int i=0;

    mean = calculate_mean(arr,n);

    for(i=0; i<n; ++i)
        std += pow(arr[i] - mean, 2);

    return sqrt(std/n);
}

int 
find_peaks_and_troughs(
		double *arr, 	// signal 
		int n_samples, 	// number of samples present in the signal
		double E, 	// threshold for peak detection
		// arrays that will store the indicies of the located
		// peaks and troughs
		double *P, double *T,
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
	E = 0.3 * calculate_range(arr, n_samples);

	while (i != n_samples) {
		i++;
		if (d == 0) {
			if (arr[a] >= (arr[i] + E)) {
				d = 2;
			} else if (arr[i] >= (arr[b] + E)) {
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


// divede each stride into 4 segments
// each segment return min, max, mean, std
double *stride_segmentation(double *arr, int n)
{
	int n_inseg = floor(n/4);
	int i = 0;

	// segment 1: arr[0] to arr[n_inseg-1]
	// segment 2: arr[n_inseg] to arr[2*n_inseg-1]
	// segment 3: arr[2*n_inseg] to arr[3*n_inseg-1]
	// segment 4: arr[3*n_inseg] to arr[n-1]

	double *segment_features; // output
	double *segment1, *segment2, *segment3, *segment4;
	segment_features = (double *) malloc(sizeof(double) * 20);
	segment1 = (double *) malloc(sizeof(double) * (n_inseg));
	segment2 = (double *) malloc(sizeof(double) * (n_inseg));
	segment3 = (double *) malloc(sizeof(double) * (n_inseg));
	segment4 = (double *) malloc(sizeof(double) * (n_inseg));

	for (i=0;i<n_inseg;i++)
	{
		segment1[i] = arr[i];
	}

	for (i=0;i<n_inseg;i++)
	{
		segment2[i] = arr[i+n_inseg];
	}

	for (i=0;i<n_inseg;i++)
	{
		segment3[i] = arr[i+2*n_inseg];
	}

	for (i=0;i<n_inseg;i++)
	{
		segment4[i] = arr[i+3*n_inseg];
	}

	segment_features[0] =  calculate_min(segment1, n_inseg);
	segment_features[1] =  calculate_max(segment1, n_inseg);
	segment_features[2] =  calculate_mean(segment1, n_inseg);
	segment_features[3] =  calculate_std(segment1, n_inseg);

	segment_features[4] =  calculate_min(segment2, n_inseg);
	segment_features[5] =  calculate_max(segment2, n_inseg);
	segment_features[6] =  calculate_mean(segment2, n_inseg);
	segment_features[7] =  calculate_std(segment2, n_inseg);

	segment_features[8] =  calculate_min(segment3, n_inseg);
	segment_features[9] =  calculate_max(segment3, n_inseg);
	segment_features[10] =  calculate_mean(segment3, n_inseg);
	segment_features[11] =  calculate_std(segment3, n_inseg);

	segment_features[12] =  calculate_min(segment4, n_inseg);
	segment_features[13] =  calculate_max(segment4, n_inseg);
	segment_features[14] =  calculate_mean(segment4, n_inseg);
	segment_features[15] =  calculate_std(segment4, n_inseg);

	return segment_features;
}

// divede each stride into 3 segments
// each segment return min, max, mean, std
double *stride_segmentation_3(double *arr, int n)
{
	int n_inseg = floor(n/3);
	int i = 0;

	// segment 1: arr[0] to arr[n_inseg-1]
	// segment 2: arr[n_inseg] to arr[2*n_inseg-1]
	// segment 3: arr[2*n_inseg] to arr[3*n_inseg-1]
	// segment 4: arr[3*n_inseg] to arr[n-1]

	double *segment_features; // output
	double *segment1, *segment2, *segment3;
	segment_features = (double *) malloc(sizeof(double) * 12);
	segment1 = (double *) malloc(sizeof(double) * (n_inseg));
	segment2 = (double *) malloc(sizeof(double) * (n_inseg));
	segment3 = (double *) malloc(sizeof(double) * (n_inseg));

	for (i=0;i<n_inseg;i++)
	{
		segment1[i] = arr[i];
	}

	for (i=0;i<n_inseg;i++)
	{
		segment2[i] = arr[i+n_inseg];
	}

	for (i=0;i<n_inseg;i++)
	{
		segment3[i] = arr[i+2*n_inseg];
	}

	segment_features[0] =  calculate_min(segment1, n_inseg);
	segment_features[1] =  calculate_max(segment1, n_inseg);
	segment_features[2] =  calculate_mean(segment1, n_inseg);
	segment_features[3] =  calculate_std(segment1, n_inseg);

	segment_features[4] =  calculate_min(segment2, n_inseg);
	segment_features[5] =  calculate_max(segment2, n_inseg);
	segment_features[6] =  calculate_mean(segment2, n_inseg);
	segment_features[7] =  calculate_std(segment2, n_inseg);

	segment_features[8] =  calculate_min(segment3, n_inseg);
	segment_features[9] =  calculate_max(segment3, n_inseg);
	segment_features[10] =  calculate_mean(segment3, n_inseg);
	segment_features[11] =  calculate_std(segment3, n_inseg);

	return segment_features;
}


// each stride as 1 segment
// each segment return min, max, mean, std
double *stride_segmentation_1(double *arr, int n)
{
    int n_inseg = n;
    
    // segment 1: arr[0] to arr[n_inseg-1]
    
    double *segment_features; // output
    //double *segment1;
    segment_features = (double *) malloc(sizeof(double) * 4);
    //segment1 = (double *) malloc(sizeof(double) * (n_inseg));
    
    segment_features[0] =  calculate_min(arr, n_inseg);
    segment_features[1] =  calculate_max(arr, n_inseg);
    segment_features[2] =  calculate_mean(arr, n_inseg);
    segment_features[3] =  calculate_std(arr, n_inseg);
    //free(segment1);
    return segment_features;
}

double *extract_features(double *t, double *y2_gyro, double *x2_accel, double *y_gyro, double *x_accel, double *z_gyro, int N_SAMPLES){
    double *P_i; 	// indicies of each peak found by peak detection
    double *T_i; 	// indicies of each trough found by trough detection
    double *S_i; 	// indicies of the start of each stride
    int n_P; 	// number of peaks
    int n_T; 	// number of troughs
    int n_S; 	// number of strides
    int i, idx;
    double *x = y2_gyro;
    
    double *features;
    features = (double *) malloc(sizeof(double) * 23);
    //double features[23];
    
    P_i = (double *) malloc(sizeof(double) * N_SAMPLES);
    T_i = (double *) malloc(sizeof(double) * N_SAMPLES);
    S_i = (double *) malloc(sizeof(double) * N_SAMPLES);
    
    double center = 0;
    double pk_threshold = 0;
    
    int rv = find_peaks_and_troughs(
                                    y2_gyro,
                                    N_SAMPLES,
                                    pk_threshold,
                                    P_i, T_i,
                                    &n_P, &n_T);
    
    int k, idx_next;
    double stride_time = 0;
    int n_P_new = n_P;
    int n_T_new = n_T;
    
    // Process Peaks
    
    for (i = 0; i < n_P_new; i++)
    {
        idx = (int) P_i[i];
        idx_next = (int) P_i[i+1];
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
    n_P = n_P_new;
    
    // Process Troughs
    for (i = 0; i < n_T_new-1; i++)
    {
        idx = (int) T_i[i];
        
        if (fabs(x[idx] - center) < 0.02)
        {
            for (k = 0; k < n_T_new-i-1; k++)
            {
                T_i[i+k] = T_i[i+k+1];
            }
            //i--;
            n_T_new--;
        }
        
        idx_next = (int) T_i[i+1];
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
    
    
    //printf("%d troughs removed\n", n_T - n_T_new);
    n_T = n_T_new;
    
    // Identify Strides
    // Strides are defined by a trough and peak in this order
    // A stride starts before a trough and ends after a peak
    
    int peak_index, trough_index;
    int edge_found;
    double time_threshold = 0.01;
    double value_threshold = 0.1;
    
    for (i = 0; i < n_P; i++) // There can be at most n_P strides
    {
        trough_index = (int) T_i[i];
        peak_index = (int) P_i[i];
        
        // For acceleration
        if (trough_index < peak_index)
        {
            // Find where the stride starts by finding a value before the trough
            edge_found = 0;
            k = 0;
            while (!edge_found && (trough_index-k > 0))
            {
                if ((fabsf(x[trough_index-k]-center) < value_threshold) && (t[trough_index] - t[trough_index-k] > time_threshold))
                {
                    S_i[n_S] = trough_index - k;
                    n_S++;
                    edge_found = 1;
                }
                k++;
            }
            
            // Find where the stride ends by finding a value after the peak
            edge_found = 0;
            k = 0;
            while (!edge_found && (peak_index+k < N_SAMPLES))
            {
                if ((fabsf(x[peak_index+k]-center) < value_threshold) && (t[peak_index+k] - t[peak_index] > time_threshold))
                {
                    S_i[n_S] = peak_index + k;
                    n_S++;
                    edge_found = 1;
                }
                k++;
            }
        }
        
        // For deceleration
        else
        {
            // Find where the stride begins by finding a value before the peak
            edge_found = 0;
            k = 0;
            while (!edge_found && (peak_index-k > 0))
            {
                if ((fabsf(x[peak_index-k]-center) < value_threshold) && (t[peak_index] - t[peak_index-k] > time_threshold))
                {
                    S_i[n_S] = peak_index - k;
                    n_S++;
                    edge_found = 1;
                }
                k++;
            }
            
            // Find where the stride ends by finding a value after the trough
            edge_found = 0;
            k = 0;
            while (!edge_found && (trough_index+k < N_SAMPLES))
            {
                if ((fabsf(x[trough_index+k]-center) < value_threshold) && (t[trough_index+k] - t[trough_index] > time_threshold))
                {
                    S_i[n_S] = trough_index + k;
                    n_S++;
                    edge_found = 1;
                }
                k++;
            }
            
        }
        
    }
    
    /* Write the stride features to the training file */
    int num_samples = n_P;
    double scale_slope = 1.00; // change from 0.25 to 1 because it's already normalized
    double scale_offset = 0.00;
    double period_slope = 1; // nomalize the period
    double period_offset = 0.00;
    int peak_idx1 = 0, peak_idx2 = 0;
    int trough_idx = 0, trough_idx1 = 0, trough_idx2 = 0;
    k = 0;
    // print header
    num_samples = n_P;
    
    double *minima_gz, *maxima_gz, *period_gz;
    minima_gz = (double *) malloc(sizeof(double) * N_SAMPLES);
    maxima_gz = (double *) malloc(sizeof(double) * N_SAMPLES);
    period_gz = (double *) malloc(sizeof(double) * N_SAMPLES);
    
    for (i = 0; (i < n_P) && (k < n_S); i++)
    {
        
        peak_idx1 = (int) P_i[i];
        peak_idx2 = (int) P_i[i+1];
        trough_idx = (int) T_i[i];
        trough_idx1 = (int) T_i[i];
        trough_idx2 = (int) T_i[i+1];
        
        minima_gz[i] = scale_slope * x[trough_idx] + scale_offset;                            // minima is the trough
        maxima_gz[i] = scale_slope * x[peak_idx1] + scale_offset;                             // maxima is the peak
        period_gz[i] = period_slope * (t[trough_idx2] - t[trough_idx1]) + period_offset;          // period is the time from trough to trough
        
        // stride segmentation
        double *segment_start = y2_gyro, *segment_map = x2_accel, *segment_map2 = y_gyro, *segment_map3 = x_accel, *segment_map4 = z_gyro;
        int m;
        for (m = 0; m <= trough_idx1; m++)
        {
            segment_start++;
            segment_map++;
            segment_map2++;
            segment_map3++;
            segment_map4++;
        }
        
        int segment_length = 0;
        segment_length = trough_idx2 - trough_idx1 +1;
        
        // do i need to allocate memory here?
        double *segmentation_result;
        segmentation_result = stride_segmentation_1(segment_start, segment_length);
        
        
        // map gyro_z to y_accel
        // y_accel[trough_idx1] to y_accel[trough_idx2]
        double *segmentation_map_result, *segmentation_map_result2, *segmentation_map_result3, *segmentation_map_result4;
        segmentation_map_result = stride_segmentation_1(segment_map, segment_length);
        segmentation_map_result2 = stride_segmentation_1(segment_map2, segment_length);
        segmentation_map_result3 = stride_segmentation_1(segment_map3, segment_length);
        segmentation_map_result4 = stride_segmentation_1(segment_map4, segment_length);
        
        features[0] = minima_gz[i];
        features[1] = maxima_gz[i];
        features[2] = period_gz[i];
        int k=0;
        for(k=0; k<4;k++) features[k+3] = segmentation_result[k];
        for(k=0; k<4;k++) features[k+7] = segmentation_map_result[k];
        for(k=0; k<4;k++) features[k+11] = segmentation_map_result2[k];
        for(k=0; k<4;k++) features[k+15] =  segmentation_map_result3[k];
        for(k=0; k<4;k++) features[k+19] =  segmentation_map_result4[k];
    }
    free(P_i);
    free(T_i);
    free(S_i);
    return features;
};
