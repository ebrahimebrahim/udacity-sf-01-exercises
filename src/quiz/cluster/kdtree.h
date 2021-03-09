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

	// Destroying a node cleans up all its descendants
	~Node() {
		if (left!=NULL) { 
			delete left;
			left=NULL;
		}
		if (right!=NULL) {
			delete right;
			right=NULL;
		}
	}
};

static void insert_recurse(Node* & node, int depth, const std::vector<float> & point, int id) {
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

// This is currently set up for 3D points.
// For 3D points, change Eigen::Vector3f to Eigen::Vector2f
static void search_recurse(const Node * node, std::vector<int> & ids, int depth, const std::vector<float> & target, float distanceTol) {
	if (node==NULL) return;
	auto comparison_dim = depth % target.size();
	bool within_left = (node->point[comparison_dim] > target[comparison_dim]-distanceTol);
	bool within_right = (node->point[comparison_dim] < target[comparison_dim]+distanceTol);

	if (within_left && within_right) {
		bool within_box = true;
		for (int i=0; i<target.size(); ++i) {
			if ((node->point[i] <= target[i]-distanceTol) || (node->point[i] >= target[i]+distanceTol)) {
				within_box = false;
				break;
			}
		}
		if (within_box) {
			if ((Eigen::Vector3f(target.data()) - Eigen::Vector3f(node->point.data())).squaredNorm() < distanceTol*distanceTol)
				ids.push_back(node->id);
		}
	}

	if (within_left)
		search_recurse(node->left, ids, depth+1, target, distanceTol);
	if (within_right)
		search_recurse(node->right, ids, depth+1, target, distanceTol);
	
}



struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree() {
		// The nodes are allocated on the heap, not using smart ptrs. So tree needs to recursively clean up the nodes.
		// Pay attention to the dtor Node::~Node()
		if (root!=NULL) {
			delete root;
			root = NULL;
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_recurse(root,0,point,id);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float> & target, float distanceTol) const
	{
		std::vector<int> ids;
		search_recurse(root, ids, 0, target, distanceTol);
		return ids;
	}

};




