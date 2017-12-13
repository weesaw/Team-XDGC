//
//  main.c
//  ctest
//
//  Created by gaozhanxi on 12/10/17.
//  Copyright (c) 2017 gaozhanxi. All rights reserved.
//

#include <stdio.h>
#include <string.h>
/* for exit() */
#include <stdlib.h>
/* for fabsf() */
#include <math.h>
#include "functions.h"

#define BUFF_SIZE 1024


int main(int argc, const char * argv[]) {
    /* Generic variables */
    int i, j, idx;
    int rv;
    /* Variables for reading file line by line */
    char *ifile_name, *ofile_pt_name, *ofile_st_name, *train_file_name;
    FILE *fpp, *fps, *fpi, *fpt;
    char *line = NULL;
    size_t len = 0;
    ssize_t read;
    int N_SAMPLES;
    
    /* Variables for storing the data and storing the return values */
    double *t_accel, *t_gyro, *x_accel, *y_accel, *z_accel; 	// variables for data collected from input file
    
    double *x2_accel, *y2_accel, *z2_accel;
    double *x_gyro, *y_gyro, *z_gyro;
    double *x2_gyro, *y2_gyro, *z2_gyro;
    double *r1, *p1, *y1, *r2, *p2, *y2;
    double *t, *x; // pointers to data of interest
    double pk_threshold = 0;	// pk-threshold value
    /* Variables for peak-trough detection */
    double *P_i; 	// indicies of each peak found by peak detection
    double *T_i; 	// indicies of each trough found by trough detection
    double *S_i; 	// indicies of the start of each stride
    int n_P; 	// number of peaks
    int n_T; 	// number of troughs
    int n_S; 	// number of strides
    
    // Centers
    double center = 0;
    double center1 = 0;
    double center2 = 0;
    double center3 = 0;
    
    // Threshold Values
    double threshold_accelX=0;
    double threshold_accelY=0;
    double threshold_accelZ=0;
    double threshold_gyroX=0;
    double threshold_gyroY=0;
    double threshold_gyroZ=0;
    
    // Mode determines which data set we are using
    double mode=0;
    

    n_S = 0;
    

    ifile_name = "/Users/gaozhanxi/Desktop/EE 249A Project/Lifting1.csv";
    //ifile_name = "/Users/gaozhanxi/Desktop/EE 249A Project/ctest/ctest/Punching1.csv";
    ofile_pt_name = "/Users/gaozhanxi/Desktop/pd.csv";
    ofile_st_name = "/Users/gaozhanxi/Desktop/sd.csv";
    train_file_name = "/Users/gaozhanxi/Desktop/trd_bar.txt";
    fpi = fopen(ifile_name, "r");
    
    /* count the number of lines in the file */
    read = getline(&line, &len, fpi); //discard header of file
    N_SAMPLES = 0;
    while ((read = getline(&line, &len, fpi)) != -1) {
        N_SAMPLES++;
    }
    fprintf(stderr,
            "read \'%d\'.\n",
            N_SAMPLES);

    
    /* go back to the start of the file so that the data can be read */
    rewind(fpi);
    read = getline(&line, &len, fpi); //discard header of file
    
    /* start reading the data from the file into the data structures */
    i = 0;
    t_accel = (double *) malloc(sizeof(double) * N_SAMPLES);
    x_accel = (double *) malloc(sizeof(double) * N_SAMPLES);
    y_accel = (double *) malloc(sizeof(double) * N_SAMPLES);
    z_accel = (double *) malloc(sizeof(double) * N_SAMPLES);
    t_gyro = (double *) malloc(sizeof(double) * N_SAMPLES);
    x_gyro = (double *) malloc(sizeof(double) * N_SAMPLES);
    y_gyro = (double *) malloc(sizeof(double) * N_SAMPLES);
    z_gyro = (double *) malloc(sizeof(double) * N_SAMPLES);
    
    x2_accel = (double *) malloc(sizeof(double) * N_SAMPLES);
    y2_accel = (double *) malloc(sizeof(double) * N_SAMPLES);
    z2_accel = (double *) malloc(sizeof(double) * N_SAMPLES);
    x2_gyro = (double *) malloc(sizeof(double) * N_SAMPLES);
    y2_gyro = (double *) malloc(sizeof(double) * N_SAMPLES);
    z2_gyro = (double *) malloc(sizeof(double) * N_SAMPLES);
    
    r1 = (double *) malloc(sizeof(double) * 1);
    p1 = (double *) malloc(sizeof(double) * 1);
    y1 = (double *) malloc(sizeof(double) * 1);
    
    r2 = (double *) malloc(sizeof(double) * 1);
    p2 = (double *) malloc(sizeof(double) * 1);
    y2 = (double *) malloc(sizeof(double) * 1);
    
    t = (double *) malloc(sizeof(double) * N_SAMPLES);
    x = (double *) malloc(sizeof(double) * N_SAMPLES);
    
    while ((read = getline(&line, &len, fpi)) != -1) {
        /* parse the data */
        //rv = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &t_accel[i], &t_gyro[i], &x_accel[i], &y_accel[i], &z_accel[i], &x_gyro[i], &y_gyro[i], &z_gyro[i]);
        
        rv = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &t_accel[i], &x_accel[i], &y_accel[i], &z_accel[i], &x_gyro[i], &y_gyro[i], &z_gyro[i], &r1[0], &p1[0], &y1[0], &x2_accel[i], &y2_accel[i], &z2_accel[i], &x2_gyro[i], &y2_gyro[i], &z2_gyro[i],&r2[0], &p2[0], &y2[0]);
        
        if (rv != 19) {
            fprintf(stderr,
                    "%s %d \'%s\'. %s.\n",
                    "Failed to read line",
                    i,
                    line,
                    "Exiting"
                    );
            exit(EXIT_FAILURE);
        }
        i++;
    }
    
    
    fclose(fpi);

    // test func
    double *bar;
    bar = extract_features(t_accel, y2_gyro, x2_accel, y_gyro, x_accel, z_gyro, N_SAMPLES);
    for(int s=0; s<23;s++) printf(" %10.8lf", bar[s]);
    
    
    
    
    //input file closed
    /*
     * From selected thresholds,
     * find indicies of peaks
     * find indicies of troughs
     */
    P_i = (double *) malloc(sizeof(double) * N_SAMPLES);
    T_i = (double *) malloc(sizeof(double) * N_SAMPLES);
    S_i = (double *) malloc(sizeof(double) * N_SAMPLES);
    
    /* open the output file to write the peak and trough data */
    //printf("Attempting to write to file \'%s\'.\n", ofile_pt_name);
    fpp = fopen(ofile_pt_name, "w");
    if (fpp == NULL) {
        fprintf(stderr,
                "Failed to write to file \'%s\'.\n",
                ofile_pt_name
                );
        exit(EXIT_FAILURE);
    }
    
    /* open the output file to write the stride data */
    //printf("Attempting to write to file \'%s\'.\n", ofile_st_name);
    fps = fopen(ofile_st_name, "w");
    if (fps == NULL) {
        fprintf(stderr,
                "Failed to write to file \'%s\'.\n",
                ofile_st_name
                );
        exit(EXIT_FAILURE);
    }
    
    /* open the output file to write the features of strides */
    //printf("Attempting to write to file \'%s\'.\n", train_file_name);
    fpt = fopen(train_file_name, "w");
    if (fpt == NULL) {
        fprintf(stderr,
                "Failed to write to file \'%s\'.\n",
                train_file_name
                );
        exit(EXIT_FAILURE);
    }
    
    double *minima_ay, *maxima_ay, *period_ay, *minima_gz, *maxima_gz, *period_gz;
    

    minima_gz = (double *) malloc(sizeof(double) * N_SAMPLES);
    maxima_gz = (double *) malloc(sizeof(double) * N_SAMPLES);
    period_gz = (double *) malloc(sizeof(double) * N_SAMPLES);

    
    for (j = 0; j < 12; j++) //runs 6 times for accel and gyro
    {
        n_S = 0; // Number of strides equals to 0 for each iteration
        t = t_accel;
        switch(j)
        {//x -> column from data
            case 10: t = t_accel;
                x = x_accel;
                center = center2;
                pk_threshold = threshold_accelX;
                break;
                
            case 1:	x = y_accel;
                center = center1;
                pk_threshold = threshold_accelY;
                break;
                
            case 2: x = z_accel;
                center = center3;
                pk_threshold = threshold_accelZ;
                break;
                
            case 3: t = t_accel;
                x = x_gyro;
                center = center1;
                pk_threshold = threshold_gyroX;
                break;
                
            case 4:	x = y_gyro;
                center = center1;
                pk_threshold = threshold_gyroY;
                break;
                
            case 5: x = z_gyro;
                center = center1;
                pk_threshold = threshold_gyroZ;
                break;
                
                // addition states for the second imu
            case 6: t = t_accel;
                x = x2_accel;
                center = center2;
                pk_threshold = threshold_accelX;
                break;
                
            case 7:	x = y2_accel;
                center = center1;
                pk_threshold = threshold_accelY;
                break;
                
            case 8: x = z2_accel;
                center = center3;
                pk_threshold = threshold_accelZ;
                break;
                
            case 9: t = t_accel;
                x = x2_gyro;
                center = center1;
                pk_threshold = threshold_gyroX;
                break;
                
            case 0:	x = y2_gyro;
                center = center1;
                pk_threshold = threshold_gyroY;
                break;
                
            case 11: x = z2_gyro;
                center = center1;
                pk_threshold = threshold_gyroZ;
                break;

        }
        
        
        
        rv = find_peaks_and_troughs(
                                    x,
                                    N_SAMPLES,
                                    pk_threshold,
                                    P_i, T_i,
                                    &n_P, &n_T);
        if (rv < 0) {
            fprintf(stderr, "find_peaks_and_troughs failed\n");
            exit(EXIT_FAILURE);
        }
        
        /* DO NOT MODIFY ANYTHING BEFORE THIS LINE */
        
        /*
         * Insert your algorithm to convert from a series of peak-trough
         * indicies, to a series of indicies that indicate the start
         * of a stride.
         */
        
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
        int num_features = 23;
        int num_speeds = 7;
        double scale_slope = 1.00; // change from 0.25 to 1 because it's already normalized
        double scale_offset = 0.00;
        double period_slope = 1; // nomalize the period
        double period_offset = 0.00; // change from -1.30 to 0
        
        int peak_idx1 = 0, peak_idx2 = 0;
        int trough_idx = 0, trough_idx1 = 0, trough_idx2 = 0;
        
        
        // Label the data sets
        switch(j)
        {
            case 0: fprintf(fpp, "Gyro Y2\n");
                break;
            case 1: fprintf(fpp, "Accel Y\n");
                break;
            case 2: fprintf(fpp, "Accel Z\n");
                break;
            case 3: fprintf(fpp, "Gyro X\n");
                break;
            case 4: fprintf(fpp, "Gyro Y\n");
                break;
            case 5: fprintf(fpp, "Gyro Z\n");
                break;
            case 6: fprintf(fpp, "Accel X2\n");
                break;
            case 7: fprintf(fpp, "Accel Y2\n");
                break;
            case 8: fprintf(fpp, "Accel Z2\n");
                break;
            case 9: fprintf(fpp, "Gyro X2\n");
                break;
            case 10: fprintf(fpp, "Accel X\n");
                break;
            case 11: fprintf(fpp, "Gyro Z2\n");

        }
        
        fprintf(fpp, "P_i,P_t,P_x,T_i,T_t,T_p\n");
        for (i = 0; i < n_P || i < n_T; i++) {
            /* Only peak data if there is peak data to write */
            if (i < n_P) {
                idx = (int) P_i[i];
                fprintf(fpp, "%d,%20.10lf,%lf,",
                        idx,
                        t[idx],
                        x[idx]
                        );
            } else {
                fprintf(fpp, ",,,");
            }
            /* Only trough data if there is trough data to write */
            if (i < n_T) {
                idx = (int) T_i[i];
                fprintf(fpp, "%d,%20.10lf,%lf\n",
                        idx,
                        t[idx],
                        x[idx]
                        );
            } else {
                fprintf(fpp, ",,\n");
            }
        }
        fprintf(fpp, "\n\n\n");
        //fclose(fpp);
        
        // Label the data sets strides
        switch(j)
        {
            case 0: fprintf(fps, "Accel X\n");
                break;
            case 1: fprintf(fps, "Accel Y\n");
                break;
            case 2: fprintf(fps, "Accel Z\n");
                break;
            case 3: fprintf(fps, "Gyro X\n");
                break;
            case 4: fprintf(fps, "Gyro Y\n");
                break;
            case 5: fprintf(fps, "Gyro Z\n");
                break;
        }
        
        fprintf(fps, "S_i,S_t,S_x\n");
        for (i = 0; i < n_S; i++) {
            idx = (int) S_i[i];
            fprintf(fps, "%d,%20.10lf,%lf\n",
                    idx,
                    t[idx],
                    x[idx]
                    );
        }
        fprintf(fps, "\n\n\n");
        //fclose(fps);
        
        
        
        if (j == 0) // map gyro2 y to accel x
        {
            
            // Write the minima, maxima, and period of the stride to the training file in this order
            k = 0;
            
            // print header
            num_samples = n_P;
            //if (num_samples > n_samples_ay) num_samples = n_samples_ay;
            fprintf(fpt, "%d %d %d", num_samples, num_features, num_speeds);
            
            
            for (i = 0; (i < n_P) && (k < n_S); i++)
            {
                //stride_idx1 = (int) S_i[k];
                //stride_idx2 = (int) S_i[k+1];
                peak_idx1 = (int) P_i[i];
                peak_idx2 = (int) P_i[i+1];
                trough_idx = (int) T_i[i];
                trough_idx1 = (int) T_i[i];
                trough_idx2 = (int) T_i[i+1];
                
                minima_gz[i] = scale_slope * x[trough_idx] + scale_offset;                            // minima is the trough
                maxima_gz[i] = scale_slope * x[peak_idx1] + scale_offset;                             // maxima is the peak
                period_gz[i] = period_slope * (t[trough_idx2] - t[trough_idx1]) + period_offset;          // period is the time from trough to trough
                // if (period_gz[i] > 1) period_gz[i] = 1;  // in case period exceeds 1
                
                //if (minima_ay[i] == 0 && maxima_ay[i]==0 && period_ay[i] == 0) continue;
                
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
                
                
                fprintf(fpt, "\n%10.8lf %10.8lf %10.8lf", minima_gz[i], maxima_gz[i], period_gz[i]);
                int k=0;
                for(k=0; k<4;k++) fprintf(fpt, " %10.8lf", segmentation_result[k]);
                for(k=0; k<4;k++) fprintf(fpt, " %10.8lf", segmentation_map_result[k]);
                for(k=0; k<4;k++) fprintf(fpt, " %10.8lf", segmentation_map_result2[k]);
                for(k=0; k<4;k++) fprintf(fpt, " %10.8lf", segmentation_map_result3[k]);
                for(k=0; k<4;k++) fprintf(fpt, " %10.8lf", segmentation_map_result4[k]);
                
                // gyro_y2_trough, peak, period, gy2min, max, mean, std, ax2min, max, mean, std, gy1min, max, mean, std, ax1min, max, mean, std;
                //fprintf(fpt, "\n");
                
                /*
                if (mode == 1) // walking speed 1
                {
                    fprintf(fpt, "%d %d %d %d %d %d %d", 1, -1, -1, -1, -1, -1, -1);
                }
                
                else if (mode == 2) // walking speed 2
                {
                    fprintf(fpt, "%d %d %d %d %d %d %d", -1, 1, -1, -1, -1, -1, -1);
                }
                
                else if (mode == 3) // walking speed 3
                {
                    fprintf(fpt, "%d %d %d %d %d %d %d", -1, -1, 1, -1, -1, -1, -1);
                }
                
                else if (mode == 4) // walking speed 4
                {
                    fprintf(fpt, "%d %d %d %d %d %d %d", -1, -1, -1, 1, -1, -1, -1);
                }
                
                else if (mode == 5) // walking speed 2
                {
                    fprintf(fpt, "%d %d %d %d %d %d %d", -1, -1, -1, -1, 1, -1, -1);
                }
                
                else if (mode == 6) // walking speed 3
                {
                    fprintf(fpt, "%d %d %d %d %d %d %d", -1, -1, -1, -1, -1, 1, -1);
                }
                
                else if (mode == 7) // walking speed 4
                {
                    fprintf(fpt, "%d %d %d %d %d %d %d", -1, -1, -1, -1, -1, -1, 1);
                }
                else
                {
                    fprintf(fpt, "%d %d %d %d %d %d %d", 0, 0, 0, 0, 0, 0, 0);
                }
                 */
                k = k + 1;
            }
        }
    }
    fclose(fpp);
    fclose(fps);
    fclose(fpt);
    //remove("data.csv");
    //remove("pd.csv");
    //remove("sd.csv");
    printf("\nWalk / Run detection successfully.\n");
    
    //remove("data.csv");
    //neural_network_result("trd.txt");
    
    /*
    free(t_accel);
    free(x_accel);
    free(y_accel);
    free(z_accel);
    free(x_gyro);
    free(y_gyro);
    free(z_gyro);
    
    free(x2_accel);
    free(y2_accel);
    free(z2_accel);
    free(x2_gyro);
    free(y2_gyro);
    free(z2_gyro);
    
    free(r1);
    free(r2);
    free(p1);
    free(p2);
    free(y1);
    free(y2);
    free(P_i);
    free(T_i);
    free(S_i);
    free(maxima_gz);
    free(minima_gz);
    free(period_gz); */
    
    return 0;
}
