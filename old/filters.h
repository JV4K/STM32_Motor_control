/*
 * filters.h
 *
 *  Created on: Oct 16, 2023
 *      Author: zhmis
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

/* __________________________________
 * 			|MEDIAN FILTER|
 * __________________________________
 */

// Function to swap two float values
void swap(float* a, float* b);

// Quick Sort partition function
int partition(float arr[], int low, int high);

// Quick Sort algorithm
void quickSort(float arr[], int low, int high);

// Structure for real-time median filter
typedef struct {
    float* buffer;
    int bufferSize;
    int currentIndex;
} MedianFilter;

// Initialize the filter
MedianFilter* initMedianFilter(int bufferSize);

// Add a new value to the filter and get the filtered median
float updateAndGetMedian(MedianFilter* filter, float newValue);

// Free memory used by the filter
void freeMedianFilter(MedianFilter* filter);

/* ____________________________________________________
 * 			|EXPONENTIAL MOVING AVERAGE FILTER|
 * ____________________________________________________
 */

// Structure for Exponential Moving Average (EMA) filter
typedef struct {
    float alpha;    // Weight factor (0 < alpha < 1)
    float previous; // Previous EMA value
} EMAFilter;

// Initialize the EMA filter
EMAFilter* initEMAFilter(float alpha, float initial);

// Update the EMA filter with a new value and return the filtered result
float updateEMA(EMAFilter* filter, float newValue);

#endif /* INC_FILTERS_H_ */
