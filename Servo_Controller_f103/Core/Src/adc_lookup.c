/*
 * adc_lookup.c
 *
 *  Created on: Oct 26, 2023
 *      Author: JV4K
 */

#include "adc_lookup.h"
#include <stdio.h>

void initLookupTable(TableEntry* table, float range_start, float range_end, float a, float b, float c) {
    float step = (range_end - range_start) / (TABLE_SIZE - 1);
    for (int i = 0; i < TABLE_SIZE; i++) {
        table[i].x = range_start + i * step;
        table[i].y = a * table[i].x * table[i].x + b * table[i].x + c;
    }
}

float binarySearch(TableEntry* table, int table_size, float x) {
    if (x <= table[0].x) {
        return table[0].y; // Request is less than or equal to the start of the range
    } else if (x >= table[table_size - 1].x) {
        return table[table_size - 1].y; // Request is greater than or equal to the end of the range
    }

    int low = 0;
    int high = table_size - 1;

    while (low <= high) {
        int mid = (low + high) / 2;
        if (x < table[mid].x) {
            high = mid - 1;
        } else if (x > table[mid].x) {
            low = mid + 1;
        } else {
            return table[mid].y; // Exact value found
        }
    }

    // Closest values
    int closest_low = (low > 0) ? (low - 1) : low;
    int closest_high = (high < table_size - 1) ? (high + 1) : high;

    // Linear approximation
    float x0 = table[closest_low].x;
    float y0 = table[closest_low].y;
    float x1 = table[closest_high].x;
    float y1 = table[closest_high].y;

    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}
