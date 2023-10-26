/*
 * adc_lookup.h
 *
 *  Created on: Oct 26, 2023
 *      Author: JV4K
 */

#ifndef INC_ADC_LOOKUP_H_
#define INC_ADC_LOOKUP_H_

#define TABLE_SIZE 300  // Размер lookup-таблицы

typedef struct {
    float x;
    float y;
} TableEntry;

void initLookupTable(TableEntry* table, float range_start, float range_end, float a, float b, float c);
float linearApproximation(TableEntry* table, int table_size, float x);


#endif /* INC_ADC_LOOKUP_H_ */
