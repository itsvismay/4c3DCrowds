#pragma once
#include <iostream>
#include "Heap.h"


class MinHeap : public Heap 
{

public:

	MinHeap(Node* r):Heap(r){
	};

	void push(Node* n){
		// 1. level order traversal
		// 2. Find the next spot
		// 3. Insert
		// 4. Rebalance
	}

	Node* top(){
		std::cout<<"min heap top"<<std::endl;
		return this->root;
	}

	Node* pop(){
		std::cout<<"min heap pop"<<std::endl;
		return this->root;
	}
};