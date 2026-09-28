/*
 * detect_motion.c
 *
 *  Created on: Mar 13, 2025
 *      Author: 91748
 */



#include "stdio.h"
#include "math.h"


// Pre-computed mean and standard deviation for normal data (from normal.csv)
const float mean[3] = {0.0238f, -0.0743f, 0.6323f};
const float std[3]  = {0.6600f,  0.4720f, 0.2898f};

// Thresholds and scaling parameters
const float FREE_FALL_THRESHOLD = 0.5f;   	 // Free fall if net acceleration is below 0.5g
const float MAHA_THRESHOLD      = 3.0f;      // Mahalanobis distance threshold for abnormality
const float SCORE_SCALING       = 10.0f;     // Multiply d by 10 to get a score (max 100)
const float MAX_SCORE           = 100.0f;    // Cap the anomaly score at 100


/* Function to detect anomaly (including free fall) using Mahalanobis distance
  -->Returns 1 if abnormal, 0 if normal.
 */


int detect_motion(float x , float y , float z)
{
	// Compute Mahalanobis distance based on each axis
    float d0 = (x - mean[0]) / std[0];
    float d1 = (y - mean[1]) / std[1];
    float d2 = (z - mean[2]) / std[2];
    float d = sqrtf(d0 * d0 + d1 * d1 + d2 * d2);

    // Compute Anamoly score as a scaled version of d
    float score = d*SCORE_SCALING ;
    if (score > MAX_SCORE)
    {
    	score = MAX_SCORE ;
    }

    // Compute net acceleration to check for free fall
    float net_accel = sqrt(x*x + y*y + z*z);
    int is_free_fall = (net_accel < FREE_FALL_THRESHOLD);

    // Classify as ABNORMAL if free fall is detected or if Mahanalobis distance exceeds threshold
    int is_abnormal = ((is_free_fall) || (d > FREE_FALL_THRESHOLD)) ;

    // Print detailed report for this sample
    printf("Input: %.3f, %.3f, %.3f => Mahalanobis Distance: %.2f, Anomaly Score: %.2f/100 \r\n\n", x, y, z, d, score);

    if(is_free_fall)
    {
    	printf("Free fall detected\r\n");
    }
    else if (d > MAHA_THRESHOLD)
    {
    	printf("Abnormal activity detected \r\n");
    }
    else
    {
    	printf("Normal \r\n");
    }

    return is_abnormal ;

}

