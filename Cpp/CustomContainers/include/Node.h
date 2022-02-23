#pragma once
#include <iostream>

struct Node{
	int val;
	Node* left;
	Node* right;

	Node():val(0), left(nullptr), right(nullptr){};
	Node(int x):val(x), left(nullptr), right(nullptr){};
	Node(int x, Node* l, Node* r): val(x), left(l), right(r){};

};

template<typename T>
struct LLNode{
	T val;
	LLNode<T>* next;


	LLNode(T x):val(x), next(nullptr){};
	LLNode(T x, LLNode<T>* n): val(x), next(n){};

};