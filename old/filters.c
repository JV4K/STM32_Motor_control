/*
 * filters.c
 *
 *  Created on: Oct 16, 2023
 *      Author: zhmis
 */

#include "filters.h"

#include <stdlib.h>

/* __________________________________
 * 			|MEDIAN FILTER|
 * __________________________________
 */

// Function to swap two float values
void swap(float* a, float* b) {
    float temp = *a;
    *a = *b;
    *b = temp;
}

// Quick Sort partition function
int partition(float arr[], int low, int high) {
    float pivot = arr[high];
    int i = (low - 1);

    for (int j = low; j < high; j++) {
        if (arr[j] <= pivot) {
            i++;
            swap(&arr[i], &arr[j]);
        }
    }
    swap(&arr[i + 1], &arr[high]);
    return (i + 1);
}

// Quick Sort algorithm
void quickSort(float arr[], int low, int high) {
    if (low < high) {
        int pi = partition(arr, low, high);
        quickSort(arr, low, pi - 1);
        quickSort(arr, pi + 1, high);
    }
}

// Initialize the filter
MedianFilter* initMedianFilter(int bufferSize) {
    MedianFilter* filter = (MedianFilter*)malloc(sizeof(MedianFilter));
    filter->buffer = (float*)malloc(bufferSize * sizeof(float));
    filter->bufferSize = bufferSize;
    filter->currentIndex = 0;
    return filter;
}

// Add a new value to the filter and get the filtered median
float updateAndGetMedian(MedianFilter* filter, float newValue) {
    filter->buffer[filter->currentIndex] = newValue;
    filter->currentIndex = (filter->currentIndex + 1) % filter->bufferSize;

    // Create a copy of the buffer for sorting
    float* sortedBuffer = (float*)malloc(filter->bufferSize * sizeof(float));
    for (int i = 0; i < filter->bufferSize; i++) {
        sortedBuffer[i] = filter->buffer[i];
    }

    // Sort the buffer using Quick Sort
    quickSort(sortedBuffer, 0, filter->bufferSize - 1);

    // Get the median value
    float median;
    if (filter->bufferSize % 2 == 0) {
        // If the buffer size is even, take the average of the two middle values
        median = (sortedBuffer[filter->bufferSize / 2 - 1] + sortedBuffer[filter->bufferSize / 2]) / 2.0;
    } else {
        // If the buffer size is odd, take the middle value
        median = sortedBuffer[filter->bufferSize / 2];
    }

    free(sortedBuffer);
    return median;
}

// Free memory used by the filter
void freeMedianFilter(MedianFilter* filter) {
    free(filter->buffer);
    free(filter);
}

/* ____________________________________________________
 * 			|EXPONENTIAL MOVING AVERAGE FILTER|
 * ____________________________________________________
 */

// Initialize the EMA filter and return a pointer to it
EMAFilter* initEMAFilter(float alpha, float initial) {
    EMAFilter* filter = (EMAFilter*)malloc(sizeof(EMAFilter));
    if (filter != NULL) {
        filter->alpha = alpha;
        filter->previous = initial;
    }
    return filter;
}

// Update the EMA filter with a new value and return the filtered result
float updateEMA(EMAFilter* filter, float newValue) {
    if (filter != NULL) {
        float ema = filter->alpha * newValue + (1 - filter->alpha) * filter->previous;
        filter->previous = ema;
        return ema;
    }
    return 0.0; // Handle errors appropriately
}
