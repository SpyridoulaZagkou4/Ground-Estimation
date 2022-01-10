#include <VVRScene/canvas.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include <MathGeoLib.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <set>
#include "symmetriceigensolver3x3.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <vector>

#define FLAG_SHOW_AXES       1
#define FLAG_SHOW_WIRE       2
//#define FLAG_SHOW_SOLID      4
#define FLAG_SHOW_NORMALS    8
//#define FLAG_SHOW_PLANE     16
#define FLAG_SHOW_AABB      32

#define FLAG_SHOW_SPHERE    4                            //s
#define FLAG_SHOW_PTS_SPHERE   16                        //p
#define FLAG_SHOW_KNN      64                            //k

#define downsampling 1


std::vector<vvr::LineSeg3D> mynormals;
std::vector<vvr::Point3D> mycur;
std::vector<vvr::Point3D> region_growing;
std::vector<vvr::Point3D> m_obj;
std::vector<float> my_curvatures;
std::vector<float> mydirs_x;
std::vector<float> mydirs_y;
std::vector<float> mydirs_z;
float m_curvature;

//std::vector<vec> convex_hull_points;
vvr::Mesh m_object;
//std::vector<vec> testarw;


struct Point_Of_Interest
{
	vec m_vertice;
	vec normal;
	float curvature;
	//int proccessed = 0;
	Point_Of_Interest(vec v,vec n,float cur):
		m_vertice(v), normal(n),curvature(cur){}

};

struct point_and_box {
	int idx;
	vec m_vec;
	//point_and_box(int arg_idx) { idx = arg_idx; box = -1; }
	//bool operator < (const point_and_box& rhs) const { return(box < rhs.box); }
	point_and_box(int voxel, vec point) :
		idx(voxel), m_vec(point) {}

};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct KDNode
{
	vec split_point;
	int axis;
	int level;
	AABB aabb;
	KDNode* child_left;
	KDNode* child_right;
	KDNode() : child_left(NULL), child_right(NULL) {}
	~KDNode() { delete child_left; delete child_right; }
};

class KDTree
{
public:
	KDTree(VecArray& pts);
	~KDTree();
	std::vector<KDNode*> getNodesOfLevel(int level);
	int depth() const { return m_depth; }
	const KDNode* root() const { return m_root; }
	const VecArray& pts;

private:
	static int makeNode(KDNode* node, VecArray& pts, const int level);
	static void getNodesOfLevel(KDNode* node, std::vector<KDNode*>& nodes, int level);

private:
	KDNode* m_root;
	int m_depth;
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class Mesh3DScene : public vvr::Scene
{
public:
	Mesh3DScene();
	const char* getName() const { return "3D Scene"; }
	void keyEvent(unsigned char key, bool up, int modif) override;
	void arrowEvent(vvr::ArrowDir dir, int modif) override;
	void load_point_cloud();

	std::vector<Point_Of_Interest> mynewarray;
	std::vector<std::vector<Point_Of_Interest> >  R;
	void Task_Draw_PCA(vec& center, vec& dir);
	void draw_Curvature(vec& cur, float& curvature);

private:
	void draw() override;
	void reset() override;
	void resize() override;
	void Tasks();
	void Task_DownSamling_PointCloud(std::vector<vec>& vertices,vvr:: Box3D& aabb, math::vec vSize, std::vector<vec>& outCloud);



	
private:
	int m_style_flag;
	float m_plane_d;
	vvr::Canvas2D m_canvas;
	vvr::Colour m_obj_col;
	vvr::Mesh m_model_original, m_model;
	vvr::Box3D m_aabb;
	math::vec m_center_mass;
	math::vec m_pca_cen;
	math::vec m_pca_dir;
	math::Plane m_plane;
	std::vector<int> m_intersections;
	std::vector<vec> point_cloud;  //ta shmeia mou
	std::vector<vec> outCloud;
	std::vector<vec> m_normals;
	//VecArray point_cloud;
	//VecArray outCloud;
	math::vec m_curv;
	math::vec princ;
	//math::vec princ2;
	float m_mean;
	std::vector<vec> my_means;
	float m_gaussian;
	std::vector<vec> my_gaussians;
	std::vector<vec> mydirs;
	float m_curvature;
	
	std::vector<vec> sorted_point_cloud;

	C2DPoint i;

	//ta kainourgia stoixeia pou prosthetw                 // *
	KDTree* m_KDTree;
	KDTree* m_KDTree_new;

	KDTree* m_object_KDTree;
	int m_current_o_tree_level;
	float m_o_tree_invalidation_sec;

	//VecArray m_pts;
	vvr::Sphere3D m_sphere;
	vvr::Sphere3D m_sphere_new;
	vvr::Sphere3D m_sphere_new_1;
	vvr::Sphere3D m_sphere_new_2;
	vvr::Sphere3D m_sphere_new_3;
	vvr::Animation m_anim;
	int m_flag;
	math::LCG m_lcg;
	int m_current_tree_level;
	float m_tree_invalidation_sec;
	int m_kn;

	//vvr::Mesh m_object;
	//std::vector<vec> testarw;

	bool idle() override;
};


struct VecComparator {
	unsigned axis;
	VecComparator(unsigned axis) : axis(axis % 3) {}
	virtual inline bool operator() (const vec& v1, const vec& v2) {
		return (v1.ptr()[axis] < v2.ptr()[axis]);
	}
};

/**
 * Find all the points under `root` node of the tree.
 */
void Task_01_FindPtsOfNode(const KDNode* root, VecArray& pts);

// Geitones mesa sth  sfaira
/**
 * Find the points of `kdtree` that are contained inside `sphere`.
 */
void Task_Neighboors_InSphere(const Sphere& sphere, const KDNode* root, VecArray& pts);

/**
 * Find the `k` nearest neighbours of `test_pt` inside `root`.
 */
void Task_Find_NearestK(const int k, const vec& test_pt, const KDNode* root, const KDNode** knn, float* best_dist);


void pca(std::vector<vec>& vertices, vec& center, vec& dir);

void new_pca(std::vector<vec>& vertices, vec& center, float& mean, float& gaussian, vec& principal);



//void Task_Draw_new_PCA(vec& center, vec& princ);

bool wayToSort(float a, float b);
bool wayToSort2(Point_Of_Interest a, Point_Of_Interest b);
bool wayToSort3(Point_Of_Interest a, Point_Of_Interest b);
bool sort_indices(point_and_box a, point_and_box b);
float find_Curvature(std::vector<vec>& vertices, vec& center, vec& dir);
//void Region_Growing(std::vector<Point_Of_Interest> mynewarray, std::vector<std::vector<Point_Of_Interest> >  R, std::vector<vec> sorted_point_cloud, KDTree* m_KDTree, vvr::Sphere3D m_sphere_new_1);
void Triangulate_Objects(std::vector<vec> object, int m_kn, KDTree* m_object_KDTree, int m_current_o_tree_level, vvr::Mesh& model);
bool Is_On_Right(vec q, vec p, vec t);
std::vector<vec> convex_hull(std::vector<vec> input, std::vector<vec> convex_hull_points);
void Add_To_Mesh(std::vector<vec> c_h_points);