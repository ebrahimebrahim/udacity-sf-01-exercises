/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

void insert_recurse(Node* & node, int depth, const std::vector<float> & point, int id) {
	if (node==NULL){
		node = new Node(point, id);
	}
	else {
		auto comparison_dim = depth % point.size();
		if (point[comparison_dim] < node->point[comparison_dim])
			insert_recurse(node->left, depth+1, point, id);
		else
			insert_recurse(node->right, depth+1, point, id);
	}
}

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_recurse(root,0,point,id);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




