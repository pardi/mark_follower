#include "SortList.h"

// Sorted list constructor
SortList::SortList(){
	first_node_ = NULL;
}

// Sorted list semi-copy constructor
SortList::SortList(const cv::Point target, const double value, const bool is_square){

}

// Binary Tree destructor
SortList::~SortList(){

	clear();
	
	delete first_node_;
}
// Push element in the tree
void SortList::push(const cv::Point target, const double value, const bool is_square){
	
	// Only one node

	if (first_node_ == NULL){

		first_node_ = new NodeSL(target, value, is_square);

		return;
	}

	// Pointer
	NodeSL* p = first_node_;
	// Last Pointer
	NodeSL* pl = first_node_;

	while ( (p != NULL) && (value < p->value_) ) {
		pl = p;
		p = p->next_;
	}

	// Head push
	if (p == pl){
		pl = new NodeSL(target, value, is_square); 
		pl->next_ = p;
		first_node_ = pl;
	}
	else{
		// Normal Push
		if ((pl != NULL) && (p != NULL)){
			pl->next_ = new NodeSL(target, value, is_square);
			pl->next_->next_ = p;
		}
		else{ // Tail push
			p = new NodeSL(target, value, is_square);
			pl->next_ = p;
		}

	}

}

// Search max element
cv::Point SortList::get_max(bool* is_square){

	if (empty())
		return cv::Point(-1, -1);
	
	*is_square = first_node_->is_square_;

	return first_node_->target_;
}


// Search min element
cv::Point SortList::get_min(){

	if (empty())
		return cv::Point(-1, -1);

	// Pointer
	NodeSL* p = first_node_;
	// Last Pointer
	NodeSL* pl = first_node_;

	while (p->next_ != NULL){
		pl = p;
		p = p->next_;
	}

	return pl->target_;
}

bool SortList::empty(){

	return (first_node_ == NULL);
}

void SortList::clear(){

	// Pointer
	NodeSL* p = first_node_;
	// Last Pointer
	NodeSL* pl = first_node_;

	if (empty())
		return;

	while(p->next_ != NULL){
		pl = p;
		p = p->next_;
		delete pl;
	}

	delete p;
	
	first_node_ = NULL;
}

void SortList::print(){

	// Pointer
	NodeSL* p = first_node_;

	do{
		cout << "x: " << p->target_.x << " y: " << p->target_.y << " v: " << p->value_ << "is_square:" << p->is_square_ << endl;
		p = p->next_;
	}while(p != NULL);
	
}