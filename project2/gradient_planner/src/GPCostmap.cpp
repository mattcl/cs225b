#include <stdlib.h>
#include "GPCostmap.h"
#include "math.h"

GPCostmap::GPCostmap(int rows, int cols) {
	numRows = rows;
	numCols = cols;
	data = new float[numRows * numCols];
}

GPCostmap::~GPCostmap() {
	delete [] data;
	data = NULL;
}

int GPCostmap::getNumRows() {
	return numRows;
}

int GPCostmap::getNumCols() {
	return numCols;
}

int GPCostmap::getNumElements() {
	return numRows * numCols;
}

void GPCostmap::set(int index, float value) {
	data[index] = value;
}

void GPCostmap::set(int row, int col, float value) {
	data[toIndex(row, col)] = value;
}

void GPCostmap::set(struct coord_t coord, float value) {
	set(coord.row, coord.col, value);
}

float GPCostmap::get(int row, int col) {
	return data[toIndex(row, col)];
}

float GPCostmap::get(struct coord_t coord) {
	return get(coord.row, coord.col);
}

float* GPCostmap::getData() {
	return data;
}

void GPCostmap::loadFromArray(float* arr) {
	for(int i = 0; i < numRows * numCols; i++)
		data[i] = arr[i];
}

void GPCostmap::zero() {
	setAll(0);
}

void GPCostmap::setAll(float value) {
	for(int i = 0; i < numRows * numCols; i++) 
		data[i] = value;
}

void GPCostmap::mapAll(void (*fn)(float & value)) {
	for(int i = 0; i < numRows * numCols; i++)
		fn(data[i]);
}

float GPCostmap::interpolate(float row, float col) {
	int row_max = (int) ceil(row);
	int col_max = (int) ceil(col);
	int row_min = (int) floor(row);
	int col_min = (int) floor(col);
	float x = row - row_min;
	float y = col - col_min;
	return get(row_min, col_min) * (1 - x) * (1 - y) + get(row_max, col_min) * x * (1 - y) + get(row_min, col_max) * (1 - x) * y + get(row_max, col_max) * x * y;
}

// converts row and column to an index in a linear array
int GPCostmap::toIndex(int row, int col) {
	return col * numRows + row;
}

