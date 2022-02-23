#include <iostream>
#include <string>

#include "LLMinHeap.h"

int main(){

	LLNode<double> one = LLNode<double>(1);
	LLNode<double> two = LLNode<double>(2.3);
	LLNode<int> three = LLNode<int>(3);
	LLNode<double> four = LLNode<double>(4);
	LLNode<double> five = LLNode<double>(5);
	LLNode<double> six = LLNode<double>(6);


	LLMinHeap h = LLMinHeap(&one); 
	h.print();
	h.push(&three);
	h.print();
	h.push(&two);
	h.print();
}