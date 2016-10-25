#include "KDTreeDescObj.h"


KDTreeDescObj::KDTreeDescObj(){

	first_node_ = NULL;
	numNode = -1;
}

// KDTreeDescObj::KDTreeDescObj(const cv::Point target, const colors color, const shapes shape, const float area ){

// 	numNode++;

// 	first_node_ = new NodeObj(target, color, shape, area, numNode, ros::Time::now());

// }
		
KDTreeDescObj::~KDTreeDescObj(){

	delete first_node_;

}

void KDTreeDescObj::push(const cv::Point target, const colors color, const shapes shape, const float area){

	bool i = true;
	bool dir = 0;

	// Begin of the Tree

	NodeObj* p = first_node_;
	NodeObj* p_next = first_node_;
	// Loop until a leaf is found

	if (first_node_ == NULL){
		numNode++;
		first_node_ = new NodeObj(target, color, shape, area, numNode, ros::Time::now());
		return;
	}

	while(p_next != NULL){

		p = p_next;

		if (i == true){
			if (target.x > p_next->target_.x){
				p_next = p_next->right_; 
				dir  = false;
			}
			else{
				p_next = p_next->left_;
				dir  = true;
			}
		}
		else{ // i == false
			if (target.y > p_next->target_.y){
				p_next = p_next->right_; 
				dir  = false;
			}
			else{
				p_next = p_next->left_;
				dir  = true;
			}
		}

		i != i;

	}

	// Add new node
	numNode++;

	if (dir)
		p->left_ = new NodeObj(target, color, shape, area, numNode, ros::Time::now());
	else
		p->right_ = new NodeObj(target, color, shape, area, numNode, ros::Time::now());


}

cv::Point KDTreeDescObj::nearestObj(const cv::Point target, const colors color, const shapes shape, const float area){

	// Begin of the tree
	NodeObj* p = first_node_;
	NodeObj* p_next = first_node_;

	bool i =0;

	while(p_next != NULL){

		p = p_next;

		if (i == true){
			if (target.x > p_next->target_.x)
				p_next = p_next->right_; 
			else
				p_next = p_next->left_;
		}
		else{ // i == false
			if (target.y > p_next->target_.y)
				p_next = p_next->right_; 
			else
				p_next = p_next->left_;
		}

		i != i;

	}

	return p->target_;
}


vector<NodeObj> KDTreeDescObj::neighbourhoodObj(const cv::Point target, const float bound, NodeObj* KDTree){

	if (KDTree == NULL)
		return recursiveNeighbourhoodObj(target, bound, first_node_, true);

	return recursiveNeighbourhoodObj(target, bound, KDTree, true);
}

vector<NodeObj> KDTreeDescObj::recursiveNeighbourhoodObj(const cv::Point target, const float bound, NodeObj* KDTree, bool layer){

	vector<NodeObj> vect;

	// Am I a dead breach?
	if (KDTree == NULL)
		return vect;

	// Am I a leaf?
	if (KDTree->right_ == NULL && KDTree->left_ == NULL){
		
		vect.push_back(*KDTree);
		return vect;
	}

	int diff;

	// Choose right layer
	if (layer == true)
		diff = target.x - KDTree->target_.x;
	else
		diff = target.y - KDTree->target_.y;

	// cout << KDTree->target_ << endl;

	// Investigate on right or left branch of the tree

	if (abs(diff) > bound ){

		cout << "separto" << endl;

		if (diff > 0)
			vect = recursiveNeighbourhoodObj(target, bound, KDTree->right_, !layer );
		else
			vect = recursiveNeighbourhoodObj(target, bound, KDTree->left_, !layer );
	}else{
		cout << "completoL"<< endl;
		vect = 	recursiveNeighbourhoodObj(target, bound, KDTree->left_, !layer );
	
		vector<NodeObj> vect1;
		// cout << "completoR"<< endl;
		for (int i = 0; i < vect.size(); i++)
			cout << vect[i].target_ << " ";
		cout<< endl;


		cout << KDTree->target_ << endl;
		// vect1 = recursiveNeighbourhoodObj(target, bound, KDTree->right_, !layer );
		// for (int i = 0; i < vect1.size(); i++)
		// 	cout << vect1[i].target_ << " ";
		// cout<< endl;

		// vect.insert( vect.end(), vect1.begin(), vect1.end() );

	}

	if (norm(target - KDTree->target_) < bound){

		cout << KDTree->target_ << endl;
		cout << "NO segm" << endl;
		vect.push_back(*KDTree);

		cout << "UP" << endl;

	}

	return vect;
}




void KDTreeDescObj::updateKalman(){

}

bool KDTreeDescObj::empty(){

	return (first_node_ == NULL);
}

// void KDTreeDescObj::clear(){

// 	NodeObj* p;

// }

void KDTreeDescObj::print(){

	recursivePrint(first_node_);
}

void KDTreeDescObj::recursivePrint(NodeObj* obj){

	if (obj == NULL)
		return;

	// Print object
	singleNodePrint(obj);

	// Print left branch
	recursivePrint(obj->left_);

	//Print right branch
	recursivePrint(obj->right_);
}

void KDTreeDescObj::singleNodePrint(NodeObj* obj){

	if (obj == NULL){
		cout << "Empty Object" << endl;
		return;
	}

	printf("[t: (%d, %d)  c: %c s: %c  a:%f id:%d]\n", 	obj->target_.x, 
														obj->target_.y, 
														(obj->color_ == BLUE)?'B':(obj->color_ == RED)?'R':'b',
														(obj->shape_ == SQUARE)?'S':(obj->shape_ == CIRCLE)?'C':'R',
														obj->area_,
														obj->ID_
														);
}
