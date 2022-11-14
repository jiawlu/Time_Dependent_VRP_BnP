#pragma once

#include <iostream>
#include <vector>

void ProgramStop();


std::vector<std::string> split(const std::string &str, const std::string &pattern);


template <typename T>
T** AllocateDynamicArray(int nRows, int nCols)
{
	T** dynamicArray;

	dynamicArray = new (std::nothrow) T*[nRows];

	if (!dynamicArray)
	{
		std::cout << "Error: insufficient memory.";
		ProgramStop();
	}

	for (int i = 0; i < nRows; ++i)
	{
		dynamicArray[i] = new (std::nothrow) T[nCols];

		if (!dynamicArray[i])
		{
			std::cout << "Error: insufficient memory.";
			ProgramStop();
		}
	}

	return dynamicArray;
}

template <typename T>
void DeallocateDynamicArray(T** dArray, int nRows, int nCols)
{
	if (!dArray)
		return;

	for (int x = 0; x < nRows; ++x)
		delete[] dArray[x];

	delete[] dArray;
}

template <typename T>
T*** Allocate3DDynamicArray(int nX, int nY, int nZ)
{
	T*** dynamicArray = new (std::nothrow) T**[nX];

	if (!dynamicArray)
	{
		std::cout << "Error: insufficient memory.";
		ProgramStop();
	}

	for (int x = 0; x < nX; ++x)
	{
		dynamicArray[x] = new (std::nothrow) T*[nY];

		if (!dynamicArray[x])
		{
			std::cout << "Error: insufficient memory.";
			ProgramStop();
		}

		for (int y = 0; y < nY; ++y)
		{
			dynamicArray[x][y] = new (std::nothrow) T[nZ];
			if (!dynamicArray[x][y])
			{
				std::cout << "Error: insufficient memory.";
				ProgramStop();
			}
		}
	}

	for (int x = 0; x < nX; ++x)
		for (int y = 0; y < nY; ++y)
			for (int z = 0; z < nZ; ++z)
				dynamicArray[x][y][z] = 0;

	return dynamicArray;
}

template <typename T>
void Deallocate3DDynamicArray(T*** dArray, int nX, int nY)
{
	if (!dArray)
		return;

	for (int x = 0; x < nX; ++x)
	{
		for (int y = 0; y < nY; ++y)
			delete[] dArray[x][y];

		delete[] dArray[x];
	}

	delete[] dArray;
}