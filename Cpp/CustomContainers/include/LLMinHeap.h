#pragma once
#include <iostream>
#include "Node.h"

template<typename T>
class LLMinHeap
{
	LLNode<T>* head;

public:
	LLMinHeap(LLNode<T>* n){
		head = n;
	};

	void push(LLNode<T>* o){
		LLNode<T>* n = head;
		if(n==nullptr){
			head = o;
			return;
		}
		LLNode<T>* p = n;
		n = p->next;
		while(n != nullptr){
			if(o->val <= n->val){
				p->next = o;
				o->next = n;
				return;
			}
			p = n;
			n = p->next;
		}
		p->next = o;
		o->next = n;
		return;

	}

	LLNode<T>* top(){
		return head;
	}

	LLNode<T>* pop(){
		if(head == nullptr){
			throw std::invalid_argument("Cannot pop from empty");
		}
		LLNode<T>* newHead = head->next;
		head = newHead;
		return head;

	}

	void print(){
		LLNode<T>* n = head;
		while(n != nullptr){
			//min heap, so insert if o<n
			std::cout<<n->val<<", ";
			n = n->next;
		}
		std::cout<<std::endl;
	}

	~LLMinHeap(){
		// LLNode* n = head;
		// while(n->next != nullptr)
		// {
		// 	LLNode* m = n;
		// 	n = n->next;
		// 	delete m;
		// }
		// delete n;

	}
};