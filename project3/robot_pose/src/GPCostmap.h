#ifndef _gpcostmap_h
#define _gpcostmap_h


struct coord_t {
	int row;
	int col;
	float value;
};

class CompareCoord {
	public:
	bool operator()(coord_t & c1, coord_t & c2) {
		if(c1.value > c2.value) return true;
		return false;
	}
};

class GPCostmap {
public:
	GPCostmap(int rows, int cols);
	~GPCostmap();

	int getNumRows();
	int getNumCols();
	int getNumElements();
	void set(int index, float value);
	void set(int row, int col, float value);
	void set(struct coord_t coord, float value);
	float get(int row, int col);
	float get(struct coord_t coord);
	float* getData();
	void loadFromArray(float* arr);
	void zero();
	void setAll(float value);
	void mapAll(void (*fn)(float & value));
	float interpolate(float row, float col);
private:
	int numRows;
	int numCols;
	float* data;
	
	int toIndex(int row, int col);
};

#endif
