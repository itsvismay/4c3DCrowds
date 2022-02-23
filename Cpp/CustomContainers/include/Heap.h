#pragma once
#include <iostream>
#include "Node.h"

class Heap{

protected:
	Node* root;
	void preOrderTraversal(Node* r){
		if(r == nullptr){
			return;
		}

		std::cout<<r->val<<", ";
		preOrderTraversal(r->left);
		preOrderTraversal(r->right);
	}
	void inOrderTraversal(Node* r){
		if(r == nullptr){
			return;
		}

		inOrderTraversal(r->left);
		std::cout<<r->val<<", ";
		inOrderTraversal(r->right);
	}
	void postOrderTraversal(Node* r){
		if(r == nullptr){
			return;
		}

		postOrderTraversal(r->left);
		postOrderTraversal(r->right);
		std::cout<<r->val<<", ";
	}

public:
	Heap(Node* r):root(r){};
	virtual void push(Node* n){};
	virtual Node* top(){ return this->root;};
	virtual Node* pop() = 0;

	void printPreOrder()
	{
		this->preOrderTraversal(this->root);
		std::cout<<std::endl;
	}

	void printInOrder(){
		this->inOrderTraversal(this->root);
		std::cout<<std::endl;
	}

	void printPostOrder(){
		this->postOrderTraversal(this->root);
		std::cout<<std::endl;
	}
};

