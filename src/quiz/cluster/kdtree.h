/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H
#define KDTREE_H


#include "../../render/render.h"
// #include <limits>


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

struct KdTree
{
  public:	
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(const std::vector<float> point, int id){

		inserthelper(&root,point,id, X);

	}

	std::vector<int> search(const std::vector<float> target, const float distanceTol){
		std::vector<int> ids;
		searchhelper(root,target,distanceTol,ids,X);
		return ids;

	}

  private:
	const static int X{0}, Y{1}, Z{2}, I{3};

	void inserthelper(Node** node, const std::vector<float>& point, const int id, const int coordinate)
	{

		if ( *node == nullptr ){
			*node= new Node(point,id);
			return;
		}

		bool goleftbranch{ point.at(coordinate) < (*node)->point.at(coordinate)  };

		inserthelper( (goleftbranch? &(*node)->left : &(*node)->right ), point, id, (coordinate+1)%3 );

	}

	bool iswithinbox(const Node* node, const std::vector<float>& target, const float distanceTol)
	{
		float boxXLow= target.at(X)-distanceTol;
		float boxXHigh= target.at(X)+distanceTol;

		bool isWithinXBound= node->point.at(X)>=boxXLow && node->point.at(X)<=boxXHigh;

		float boxYLow= target.at(Y)-distanceTol;
		float boxYHigh= target.at(Y)+distanceTol;

		bool isWithinYBound= node->point.at(Y)>=boxYLow && node->point.at(Y)<=boxYHigh;

		float boxZLow= target.at(Z)-distanceTol;
		float boxZHigh= target.at(Z)+distanceTol;
		bool isWithinZBound= node->point.at(Z)>=boxZLow && node->point.at(Z)<=boxZHigh;

		return(isWithinXBound&&isWithinYBound&&isWithinZBound);

	}

	bool isWithinRange(const std::vector<float>& target, const std::vector<float>& nodepoint,const float distanceTol){
		float deltaX=target.at(X)-nodepoint.at(X);
		float deltaY=target.at(Y)-nodepoint.at(Y);
		float deltaZ=target.at(Z)-nodepoint.at(Z);

		float distance= sqrt(deltaX*deltaX+deltaY*deltaY+deltaZ*deltaZ);

		return(distance<=distanceTol);
	}

	void searchhelper(const Node* node, const std::vector<float>& target,  const float distanceTol, std::vector<int>& ids, const int coordinate){
		if ( node == nullptr){
			return;
		}

		if ( iswithinbox(node, target, distanceTol) ){
			if (isWithinRange (target,node->point,distanceTol) ){
				ids.push_back(node->id);
			}
			
		}

		if( target.at(coordinate)+distanceTol>node->point.at(coordinate)){
			searchhelper(node->right,target,distanceTol,ids,(coordinate+1)%3);
		}

		if( target.at(coordinate)-distanceTol<=node->point.at(coordinate)){
			searchhelper(node->left,target,distanceTol,ids,(coordinate+1)%3);
		}

	}
	

};

#endif


