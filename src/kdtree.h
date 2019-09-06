/* \authors Aaron Brown
		   Bruno Santos */
// Quiz on implementing kd tree


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
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// Fill in this function to insert a new point into the tree
		insertHelp(&root,0,point,id);
		// the function should create a new node and place correctly with in the root 

	}
	void insertHelp(Node **node,int depth, std::vector<float> point,int id){
		if((*node)==NULL)
			*node= new Node(point,id); 
		
		else{
			uint cd= depth%3;
			
			if (point[cd]< ((*node)->point[cd]))
				insertHelp(&((*node)->left),depth+1,point,id);
			
			else
				insertHelp(&((*node)->right),depth+1,point,id);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(&root,0,ids,target, distanceTol);
		return ids;
	}

	
	void searchHelper(Node **node,int depth,std::vector<int> &ids,std::vector<float> target, float distanceTol)
	{
		if((*node)!=NULL){
			if (((target[0]-distanceTol)<= (*node)->point[0]) && ((target[0]+distanceTol) >= (*node)->point[0]) && 
			((target[1]+distanceTol) >= (*node)->point[1]) && ((target[1]-distanceTol)<= (*node)->point[1]) 
			&& ((target[2]+distanceTol) >= (*node)->point[2]) && ((target[2]-distanceTol)<= (*node)->point[2])  ){
				if(distance(target,(*node)->point)<= distanceTol)
					ids.push_back((*node)->id);
			}
			uint cd= depth%3;
			//check across boundary
			if ((target[cd]-distanceTol)<=((*node)->point[cd]))
				searchHelper(&((*node)->left),depth+1,ids, target,  distanceTol);
			if((target[cd]+distanceTol)>=((*node)->point[cd]))
				searchHelper(&((*node)->right),depth+1,ids, target,  distanceTol);
			}
	}

	double distance(std::vector<float> a ,std::vector<float> b ) const
{
    const double x_diff = a[0] - b[0];
    const double y_diff = a[1] - b[1];
    const double z_diff = a[2] - b[2];
    return std::sqrt(x_diff * x_diff + y_diff * y_diff+ z_diff*z_diff);
}
};




