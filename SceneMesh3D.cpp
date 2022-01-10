#include "SceneMesh3D.h"
#include <chrono>
#include <math.h>
#include <cmath>
#include<iterator> 
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h> 
using namespace std::chrono;

#define SEC_PER_FLOOR 10
#define SPLIT_INSTEAD_OF_INTERSECT 0
#define DIMENSIONS 3
#define GND_WIDTH 40
#define SPHERE_RAD 0.5


#define DRAW_CURVATURES 0
#define MEAN_GAUSSIAN 0
#define REGION_GROWING 0
#define REMOVE_GROUND 0
#define TRIANGULATION2 0
#define TRIANGULATION1 0

using namespace std;
using namespace vvr;

Mesh3DScene::Mesh3DScene()
{
	//! Load settings.
	vvr::Shape::DEF_LINE_WIDTH = 4;
	vvr::Shape::DEF_POINT_SIZE = 3;
	m_perspective_proj = true;
	m_bg_col = Colour("768E77");

	m_KDTree = NULL;                       //*

	reset();
}

void Mesh3DScene::load_point_cloud() {

	// load point cloud
	const string pcldir = getBasePath() + "resources/obj/";
	const string pclFile = pcldir + "0000000001.bin";

	fstream input(pclFile.c_str(), ios::in | ios::binary);
	if (!input.good()) {
		cerr << "Could not read file: " << pclFile << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
	int i;
	for (i = 0; input.good() && !input.eof(); i++) {
		vec point;
		float intensity;
		input.read((char*)&point.x, 3 * sizeof(float));
		input.read((char*)&intensity, sizeof(float));

		point_cloud.push_back(point);
	}
	input.close();


}

void  Mesh3DScene::Task_DownSamling_PointCloud(vector<vec>& vertices, Box3D& aabb, vec vSize, vector<vec>& outCloud)
{
	//!//////////////////////////////////////////////////////////////////////////////////
	//! TASK:
	//!
	//!  - Breite to Axis Aligned Bounding Box tou montelou
	//!
	//! HINTS:
	//!
	//!  - To `aabb` orizetai apo 2 gwniaka simeia. (V_min, V_max)
	//!  - V_min: { aabb.x1, aabb.y1, aabb.z1 }
	//!  - V_max: { aabb.x2, aabb.y2, aabb.z2 }
	//!
	//!//////////////////////////////////////////////////////////////////////////////////

	double max_x = vertices[0].x;
	double max_y = vertices[0].y;
	double max_z = vertices[0].z;
	double min_x = vertices[0].x;
	double min_y = vertices[0].y;
	double min_z = vertices[0].z;

	int size = vertices.size();

	for (int i = 0; i < size; i++) {
		if (vertices[i].x > max_x) max_x = vertices[i].x;
		if (vertices[i].y > max_y) max_y = vertices[i].y;
		if (vertices[i].z > max_z) max_z = vertices[i].z;

		if (vertices[i].x < min_x) min_x = vertices[i].x;
		if (vertices[i].y < min_y) min_y = vertices[i].y;
		if (vertices[i].z < min_z) min_z = vertices[i].z;


	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	aabb.x1 = max_x;
	aabb.y1 = max_y;
	aabb.z1 = max_z;

	aabb.x2 = min_x;
	aabb.y2 = min_y;
	aabb.z2 = min_z;
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//cout << aabb.x1 << endl;
	vec minb;
	vec maxb;
	minb.x = floor(aabb.x2 / vSize.x);
	minb.y = floor(aabb.y2 / vSize.y);
	minb.z = floor(aabb.z2 / vSize.z);

	maxb.x = floor(aabb.x1 / vSize.x);
	maxb.y = floor(aabb.y1 / vSize.y);
	maxb.z = floor(aabb.z1 / vSize.z);
	// arithmos vovels se kathe aksona
	unsigned int numDivX = 1 + maxb.x - minb.x;
	unsigned int numDivY = 1 + maxb.y - minb.y;
	unsigned int numDivZ = 1 + maxb.z - minb.z;

	//sunolikos arithmos apo voxels
	unsigned int numVox = numDivX * numDivY * numDivZ;

	std::vector<unsigned int> indices(vertices.size());

	for (int p = 0; p < vertices.size(); p++)
	{
		unsigned int i = floor(vertices[p].x / vSize.x - minb.x);
		unsigned int j = floor(vertices[p].y / vSize.y - minb.y);
		unsigned int k = floor(vertices[p].z / vSize.z - minb.z);
		unsigned int idx;
		idx = i + j * numDivX + k * numDivX * numDivY;

		indices[p] = idx;
	}

	// Store voxel centroid in output
	// Iterate through the indices and sum values to compute centroid
	std::vector<point_and_box> m_voxels;
	for (int i = 0; i < vertices.size(); i++) {

		m_voxels.emplace_back(point_and_box(indices[i], vertices[i]));
	}

	sort(m_voxels.begin(), m_voxels.end(), sort_indices);
	srand(time(NULL));

	/* generate secret number between 1 and 10: */
	int iSecret;

	for (int cp = 0; cp < vertices.size();) {
		int i = cp + 1;
		while (i < vertices.size() && indices[cp] == indices[i]) {
			++i;
		}
		iSecret = rand() % (i - cp) + cp;
		outCloud.push_back(vertices[iSecret]);
		cp = i;
	}

	// Now iterating through the voxels
	// Normalize sums to get centroid (average)
	// Some voxels may be empty and are discarded
}

bool sort_indices(point_and_box a, point_and_box b) { return a.idx < b.idx; }

void Mesh3DScene::reset()
{
	Scene::reset();

	//! Define plane
	m_plane_d = 0;
	m_plane = Plane(vec(0, 1, 1).Normalized(), m_plane_d);

	//! Define what will be vissible by default
	m_style_flag = 0;
	//m_style_flag |= FLAG_SHOW_SOLID;
	//m_style_flag |= FLAG_SHOW_WIRE;
	m_style_flag |= FLAG_SHOW_AXES;
	m_style_flag |= FLAG_SHOW_AABB;

	//m_style_flag |= FLAG_SHOW_SPHERE;
	//m_style_flag |= FLAG_SHOW_KNN;
	//m_style_flag |= FLAG_SHOW_NORMALS;
	//m_style_flag |= FLAG_SHOW_PTS_SPHERE;
	//m_style_flag |= FLAG_SHOW_PLANE;

	 //! Define scene objects
	m_sphere = vvr::Sphere3D(-GND_WIDTH / 2, 0, 0, 05 * SPHERE_RAD, vvr::Colour::white);     //*
	m_sphere_new = vvr::Sphere3D(-GND_WIDTH / 2, 0, 0, 05 * SPHERE_RAD, vvr::Colour::white);
	m_sphere_new_1 = vvr::Sphere3D(-GND_WIDTH / 2, 0, 0, 05 * SPHERE_RAD, vvr::Colour::white);     //*
	m_sphere_new_2 = vvr::Sphere3D(-GND_WIDTH / 2, 0, 0, 05 * SPHERE_RAD, vvr::Colour::white);     //*
	m_sphere_new_3 = vvr::Sphere3D(-GND_WIDTH / 2, 0, 0, 1.5*SPHERE_RAD, vvr::Colour::white);     //*

}

void Mesh3DScene::resize()
{
	static bool first_pass = true;

	if (first_pass)
	{
		Tasks();
		first_pass = false;
	}
}

void Mesh3DScene::Tasks()
{
	m_model_original = m_model;
	load_point_cloud();

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TASK 1	~~~~~~~~~~~~~~~~~~~~~~~~
	vec m_vSize;
	m_vSize.x = 0.79;
	m_vSize.y = 0.34;
	m_vSize.z = 0.15;

	Task_DownSamling_PointCloud(point_cloud, m_aabb, m_vSize, outCloud);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~  TASK 2   ~~~~~~~~~~~~~~~~~~~~~~~~~~
	m_kn = 2;
	m_current_tree_level = 0;

	//Make my kdtree                                   //*
	delete m_KDTree;
	m_KDTree = new KDTree(outCloud);
	m_tree_invalidation_sec = -1;

	m_anim.setTime(0);

	//std::vector<Point_Of_Interest> mynewarray;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TASK 02 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
	for (int i = 0; i < outCloud.size(); i++) {
		float t = m_anim.t;
		int checked;
		LineSeg3D mynormal;
		vvr::Sphere3D sphere_moved(m_sphere);
		sphere_moved.x = outCloud[i].x;
		sphere_moved.y = outCloud[i].y;
		sphere_moved.z = outCloud[i].z;
		Sphere sphere(outCloud[i], sphere_moved.rad);


		VecArray pts_in;
		Task_Neighboors_InSphere(sphere, m_KDTree->root(), pts_in);
		//!--------------------> MAKE MY PCA-NORMALS <---------------------------------------
		pca(pts_in, m_pca_cen, m_pca_dir);

		// ------------------------> CURVATURE <---------------------------------

		m_curvature = find_Curvature(pts_in, m_pca_cen, m_pca_dir);

		if (!DRAW_CURVATURES && !REGION_GROWING) {
			//if (!isnan(m_curvature) && !isinf(m_curvature)) {
				// Apothikevw se enan struct pinaka ta adistoixa dedomena
			mynewarray.emplace_back((Point_Of_Interest(outCloud[i], m_pca_dir, m_curvature)));
			//}
		}

		if (DRAW_CURVATURES && REGION_GROWING) {
			//if (!isnan(m_curvature) && !isinf(m_curvature)) {
				// Apothikevw se enan struct pinaka ta adistoixa dedomena
			mynewarray.emplace_back((Point_Of_Interest(outCloud[i], m_pca_dir, m_curvature)));
			//}
		}

		if (DRAW_CURVATURES && !REGION_GROWING) {
			// Apothikevw se enan struct pinaka ta adistoixa dedomena
			mynewarray.emplace_back((Point_Of_Interest(outCloud[i], m_pca_dir, m_curvature)));
		}
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		//  !!!!!!!!!!!!!!!!!!!!!!!! MEAN -  GAUSSIAN
		if (DRAW_CURVATURES && MEAN_GAUSSIAN) {
			vvr::Sphere3D sphere_moved_new(m_sphere_new);
			sphere_moved_new.x = m_pca_dir.x;
			sphere_moved_new.y = m_pca_dir.y;
			sphere_moved_new.z = m_pca_dir.z;
			Sphere sphere_new(m_pca_dir, sphere_moved_new.rad);

			VecArray pts_in_new;
			Task_Neighboors_InSphere(sphere_new, m_KDTree->root(), pts_in_new);

			new_pca(pts_in_new, m_pca_cen, m_mean, m_gaussian, princ);
			if (DRAW_CURVATURES && MEAN_GAUSSIAN) {
				cout << "Mean Curvature: " << m_mean << endl;
				cout << "Gaussian Curvature: " << m_gaussian << endl;
				cout << endl;
			}
			//! Draw pca line    
			//Task_Draw_new_PCA(outCloud[i], princ);
			//Task_Draw_new_PCA(outCloud[i], princ2);
		}
	}

	std::sort(mydirs_x.begin(), mydirs_x.end(), wayToSort);
	std::sort(my_curvatures.begin(), my_curvatures.end(), wayToSort);

	//if(!DRAW_CURVATURES && !REGION_GROWING)std::sort(mynewarray.begin(), mynewarray.end(), wayToSort2);

	//if (DRAW_CURVATURES && REGION_GROWING)std::sort(mynewarray.begin(), mynewarray.end(), wayToSort3);
	for (int i = 0; i < mynewarray.size(); i++) {
		if (!DRAW_CURVATURES && !REGION_GROWING)Task_Draw_PCA(mynewarray[i].m_vertice, mynewarray[i].normal);
		if (DRAW_CURVATURES && !REGION_GROWING)draw_Curvature(mynewarray[i].m_vertice, mynewarray[i].curvature);
	}
	if (DRAW_CURVATURES && REGION_GROWING)std::sort(mynewarray.begin(), mynewarray.end(), wayToSort3);


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~> TASK 03 <----------------------------------------

	//-------------------------> MY SORTED_POINT_CLOUD ------------------> sorted_point_cloud 

	if (DRAW_CURVATURES && REGION_GROWING) {
		//Region_Growing(mynewarray, R, sorted_point_cloud, m_KDTree, m_sphere_new_1);
		for (int i = 0; i < mynewarray.size(); i++) {
			sorted_point_cloud.push_back(mynewarray[i].m_vertice);
		}

		//1os tropos	

		float max = -5;
		do {
			std::vector<Point_Of_Interest> Sc;
			std::vector<Point_Of_Interest> Rc;
			Sc.push_back(mynewarray[0]);
			Rc.push_back(mynewarray[0]);

			mynewarray.erase(mynewarray.begin());
			sorted_point_cloud.erase(sorted_point_cloud.begin());

			int count = 0;
			while (Sc.size() != 0) {
				count++;
				//cout << count << endl;
				vvr::Sphere3D sphere_moved_new_1(m_sphere_new_1);
				sphere_moved_new_1.x = Sc[0].m_vertice.x;
				sphere_moved_new_1.y = Sc[0].m_vertice.y;
				sphere_moved_new_1.z = Sc[0].m_vertice.z;
				Sphere sphere_new(Sc[0].m_vertice, sphere_moved_new_1.rad);
				std::vector<vec> Neighbors;
				Task_Neighboors_InSphere(sphere_new, m_KDTree->root(), Neighbors);

				for (int j = 0; j < Neighbors.size(); j++) {
					vec somepoint = Neighbors[j];
					auto it = std::find_if(mynewarray.begin(), mynewarray.end(), [&somepoint](const Point_Of_Interest& p) {
						return somepoint.x == p.m_vertice.x && somepoint.y == p.m_vertice.y && somepoint.z == p.m_vertice.z; });
					if (it != mynewarray.end()) {
						int index = it - mynewarray.begin();
						float angle = std::acos((Sc[0].normal.Dot(mynewarray[index].normal)) / (Sc[0].normal.Length() * mynewarray[index].normal.Length()));
						if (angle > max) {
							max = angle;
						}

						if (angle < 1.2) {
							Rc.emplace_back(mynewarray[index]);
							Point_Of_Interest temp = mynewarray[index];
							float temp2 = mynewarray[index].curvature;
							mynewarray.erase(mynewarray.begin() + index);
							sorted_point_cloud.erase(sorted_point_cloud.begin() + index);

							if (temp2 < 0.085) {
								Sc.emplace_back(temp);
							}

						}
					}


				}

				Sc.erase(Sc.begin());
			}
			R.push_back(Rc);
		} while (!mynewarray.empty() || !sorted_point_cloud.empty());
		//std::cout << "Megisthn gwnia" << max << endl;


		std::cout << R.size() << endl;
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ XRWMATISMOS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		int temp = 16777215 / R.size();   //ffffff=mavro ---> 16777217 sse dekadiko
		//cout << temp << endl;

		std::string prev_res, cur_res, fin_res;
		int decimal_value;
		std::stringstream ss;

		//new: begin
		std::vector<double> region_max_height_vector;
		double min_height = R[0][0].m_vertice.z;
		//new: end

		for (int i = 0; i < R.size(); i++) {
			ss.str("");
			decimal_value = i * temp;
			ss << std::hex << decimal_value;
			cur_res = ss.str();

			int len = 6 - cur_res.length();
			std::string zeros;
			for (int i = 0; i < len; i++) {
				zeros += "0";
			}
			fin_res = zeros + cur_res;

			Colour base_colour(fin_res);

			double region_max_height = R[i][0].m_vertice.z;	//new 

			for (int j = 0; j < R[i].size(); j++) {
				Point3D p(R[i][j].m_vertice.x, R[i][j].m_vertice.y, R[i][j].m_vertice.z, base_colour);
				region_growing.push_back(p);

				if (R[i][j].m_vertice.z < min_height) {
					min_height = R[i][j].m_vertice.z;
				}
				if (R[i][j].m_vertice.z > region_max_height) {
					region_max_height = R[i][j].m_vertice.z;
				}
			}

			region_max_height_vector.push_back(region_max_height);	//new
		}

		//cout << R.size() << endl;

			//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~> TASK 04 <----------------------------------------
		if (REMOVE_GROUND) {


			double height_thr=-0.2;	//! small_number or add small_number
			cout << "Give a heigh:" << endl;
			//cin >> height_thr;
			
			std::vector<std::vector<Point_Of_Interest>> R_obj;

			for (int i = 0; i < R.size(); i++) {
				if (region_max_height_vector[i] > height_thr) {
						//remove all points of each ground region
						R_obj.push_back(R[i]);
						//R.erase(R.begin() + i);
						
				}
			}

			std::vector<vec> m_point_obj;
			
			for (int i = 0; i < R_obj.size(); i++) {
				for (int j = 0; j < R_obj[i].size(); j++) {
					m_point_obj.push_back(R_obj[i][j].m_vertice);
				}
			}

			//new: begin
			//! Hypothesis: all ground points are removed
			m_current_tree_level = 0;
			delete m_KDTree;
			m_KDTree = new KDTree(m_point_obj);
			m_tree_invalidation_sec = -1;

			m_anim.setTime(0);

			std::vector<std::vector<vec>> objects_vector;

			do {
				std::vector<vec> Sc;
				std::vector<vec> Rc;
				Sc.push_back(m_point_obj[0]);
				Rc.push_back(m_point_obj[0]);
				m_point_obj.erase(m_point_obj.begin());
				//sorted_point_cloud.erase(sorted_point_cloud.begin());

				int count = 0;
				while (Sc.size() != 0) {
					count++;
					vvr::Sphere3D sphere_moved_new_3(m_sphere_new_3);
					sphere_moved_new_3.x = Sc[0].x;
					sphere_moved_new_3.y = Sc[0].y;
					sphere_moved_new_3.z = Sc[0].z;
					Sphere sphere_new(Sc[0], sphere_moved_new_3.rad);	//! set very small radius
					std::vector<vec> Neighbors;
					Task_Neighboors_InSphere(sphere_new, m_KDTree->root(), Neighbors);

					for (int j = 0; j < Neighbors.size(); j++) {
						vec somepoint = Neighbors[j];
						auto it = std::find_if(m_point_obj.begin(), m_point_obj.end(), [&somepoint](const vec& p) {
							return somepoint.x == p.x && somepoint.y == p.y && somepoint.z == p.z; });
						if (it != m_point_obj.end()) {
							int index = it - m_point_obj.begin();
							Rc.emplace_back(m_point_obj[index]);
							vec temp = m_point_obj[index];
							m_point_obj.erase(m_point_obj.begin() + index);
							Sc.emplace_back(temp);
						}
					}
					Sc.erase(Sc.begin());
				}
				objects_vector.push_back(Rc);
			} while (!m_point_obj.empty());
			//for each region find and draw "aabb" 
			//new: end

			temp = 16777215 / objects_vector.size();

			for (int i = 0; i < objects_vector.size(); i++) {
				ss.str("");
				decimal_value = i * temp;
				ss << std::hex << decimal_value;
				cur_res = ss.str();

				int len = 6 - cur_res.length();
				std::string zeros;
				for (int i = 0; i < len; i++) {
					zeros += "0";
				}
				fin_res = zeros + cur_res;

				Colour base_colour(fin_res);

				for (int j = 0; j < objects_vector[i].size(); j++) {
					Point3D p(objects_vector[i][j].x, objects_vector[i][j].y, objects_vector[i][j].z, base_colour);
					m_obj.push_back(p);

				}

			}
			cout << "# objects = " << R_obj.size() << endl;

			if (TRIANGULATION2) {
				for (int i = 0; i < R_obj.size(); i++) {
					std::vector<vec> testarw;
					for (int j = 0; j < R_obj[i].size(); j++) {
						testarw.push_back(R_obj[i][j].m_vertice);
					}
					std::vector<vec> convex_hull_points;
					convex_hull_points = convex_hull(testarw, convex_hull_points);
							
					if (convex_hull_points.size() >= 3)Add_To_Mesh(convex_hull_points);
					
				}
				cout << "covex hull finished" << endl;
			}

		}
		//new: end



	}
	if (TRIANGULATION1) {
		std::vector<vec> testarw;
		// ------------------------------------------------ TRIANGULATIION ---------------------------------------------
		for (int j = 0; j < R[300].size(); j++) {
			testarw.push_back(R[300][j].m_vertice);
		}
		//Triangulate_Objects(testarw, m_kn, m_object_KDTree, m_current_o_tree_level, m_object);
		m_current_o_tree_level = 0;
		delete m_KDTree;
		m_KDTree = new KDTree(testarw);
		//m_o_tree_invalidation_sec = -1;
		//m_anim.setTime(0);
		for (int w = 0; w < testarw.size(); w++) {
			float dist;
			const KDNode** nearests = new const KDNode * [m_kn];
			memset(nearests, NULL, m_kn * sizeof(KDNode*));

			for (int i = 0; i < m_kn; i++) {
				Task_Find_NearestK(i, testarw[w], m_KDTree->root(), nearests, &dist);
			}

			//for (int i = 0; i < m_kn; i++) {
			//if (!nearests[i]) continue;
			vec nn1 = nearests[0]->split_point;
			vec nn2 = nearests[1]->split_point;

			m_object.getVertices().push_back(testarw[w]);
			m_object.getVertices().push_back(nn1);
			m_object.getVertices().push_back(nn2);

			int index = m_object.getVertices().size() - 3;
			m_object.getTriangles().push_back(vvr::Triangle(&m_object.getVertices(), index, index + 1, index + 2));
			delete[] nearests;

			m_object.update();
		}
	}

}

bool wayToSort(float a, float b) { return a < b; }

bool wayToSort3(Point_Of_Interest a, Point_Of_Interest b) {
	//if (!isnan(a.curvature) && !isnan(b.curvature) ){
	return a.curvature < b.curvature;
	//} 
}

//bool wayToSort2(Point_Of_Interest a, Point_Of_Interest b) { return a.normal.x < b.normal.x; }

bool Mesh3DScene::idle()
{
	if (m_tree_invalidation_sec > 0 &&
		vvr::getSeconds() - m_tree_invalidation_sec > 0.8)
	{
		delete m_KDTree;
		m_KDTree = new KDTree(outCloud);
		m_tree_invalidation_sec = -1;
	}
	m_anim.update();
	return true;
}

void Mesh3DScene::arrowEvent(ArrowDir dir, int modif)
{
	math::vec n = m_plane.normal;
	if (dir == UP) m_plane_d += 1;
	if (dir == DOWN) m_plane_d -= 1;
	else if (dir == LEFT) n = math::float3x3::RotateY(DegToRad(1)).Transform(n);
	else if (dir == RIGHT) n = math::float3x3::RotateY(DegToRad(-1)).Transform(n);
	m_plane = Plane(n.Normalized(), m_plane_d);

	if (SPLIT_INSTEAD_OF_INTERSECT == false) {
		m_intersections.clear();

	}
	else {
		m_model = Mesh(m_model_original);

	}
}

void Mesh3DScene::keyEvent(unsigned char key, bool up, int modif)
{
	Scene::keyEvent(key, up, modif);
	key = tolower(key);

	switch (key)
	{
		//case 's': m_style_flag ^= FLAG_SHOW_SOLID; break;
	case 'w': m_style_flag ^= FLAG_SHOW_WIRE; break;
	case 'n': m_style_flag ^= FLAG_SHOW_NORMALS; break;
	case 'a': m_style_flag ^= FLAG_SHOW_AXES; break;
		//case 'p': m_style_flag ^= FLAG_SHOW_PLANE; break;
	case 'p': m_style_flag ^= FLAG_SHOW_PTS_SPHERE; break;
	case 's': m_style_flag ^= FLAG_SHOW_SPHERE; break;
	case 'b': m_style_flag ^= FLAG_SHOW_AABB; break;
	case 'k': m_style_flag ^= FLAG_SHOW_KNN; break;
	}
}

void Mesh3DScene::draw()
{
	if (!downsampling) {
		int num_pts = point_cloud.size(); // exw 118.703 shmeia
		C3DPoint center;
		center.x = 0.0;
		center.y = 0.0;
		center.z = 0.0;
		//! Draw center mass
		for (int i = 0; i < point_cloud.size(); i++) {
			Point3D(point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, vvr::Colour::darkOrange).draw();
			center.x += point_cloud[i].x;
			center.y += point_cloud[i].y;
			center.z += point_cloud[i].z;
		}
		center.x /= num_pts;
		center.y /= num_pts;
		vvr::Point3D(center.x, center.y, center.z, vvr::Colour::red).draw();

		if (m_style_flag & FLAG_SHOW_AABB) {
			m_aabb.setColour(Colour::black);
			m_aabb.setTransparency(1);
			m_aabb.draw();
		}
	}
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  TASK 01 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if (downsampling) {
		int num_pts = outCloud.size(); // exw 118.703 shmeia
		C3DPoint center;
		center.x = 0.0;
		center.y = 0.0;
		center.z = 0.0;
		//! Draw center mass
		for (int i = 0; i < outCloud.size(); i++) {
			if (!DRAW_CURVATURES)Point3D(outCloud[i].x, outCloud[i].y, outCloud[i].z, vvr::Colour::darkOrange).draw();
			center.x += outCloud[i].x;
			center.y += outCloud[i].y;
			center.z += outCloud[i].z;
		}
		center.x /= num_pts;
		center.y /= num_pts;
		//vvr::Point3D(center.x, center.y, center.z, vvr::Colour::red).draw();

		if (m_style_flag & FLAG_SHOW_AABB) {
			m_aabb.setColour(Colour::black);
			m_aabb.setTransparency(1);
			m_aabb.draw();
		}
	}

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TASK 02 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



	// ------------------------------------> PATAW TO 's' GIA NA VRW TOUS GEITONES MESA STHN SFAIRA <--------------------------------
	if (m_style_flag & FLAG_SHOW_PTS_SPHERE)
	{

		//! Animate sphere
		float t = m_anim.t;

		vvr::Sphere3D sphere_moved(m_sphere);
		sphere_moved.x = outCloud[5500].x;
		sphere_moved.y = outCloud[5500].y;
		sphere_moved.z = outCloud[5500].z;
		Sphere sphere(outCloud[5500], sphere_moved.rad);
		if (m_style_flag & FLAG_SHOW_SPHERE) {
			sphere_moved.draw();
		}
		VecArray pts_in;
		Task_Neighboors_InSphere(sphere, m_KDTree->root(), pts_in);
		math2vvr(outCloud[5500], vvr::Colour::blue).draw();


		for (auto i : pts_in) {
			math2vvr(i, vvr::Colour::magenta).draw();
		}


	}

	// ------------------------------------> PATAW TO 'k' GIA NA VRW TOUS GEITONES MESA STHN SFAIRA <--------------------------------
	if (m_style_flag & FLAG_SHOW_KNN) {
		float dist;
		const KDNode** nearests = new const KDNode * [m_kn];
		memset(nearests, NULL, m_kn * sizeof(KDNode*));

		for (int i = 0; i < m_kn; i++) {
			Task_Find_NearestK(i, outCloud[5500], m_KDTree->root(), nearests, &dist);
		}

		for (int i = 0; i < m_kn; i++) {
			if (!nearests[i]) continue;
			vec nn = nearests[i]->split_point;
			//vvr::Shape::DEF_POINT_SIZE = vvr::Shape::DEF_POINT_SIZE = POINT_SIZE;
			math2vvr(outCloud[5500], vvr::Colour::blue).draw();
			math2vvr(nn, vvr::Colour::red).draw();
			//vvr::Shape::DEF_POINT_SIZE = POINT_SIZE_SAVE;
		}
		delete[] nearests;
	}

	//------------------------------------- NORMALS -----------------------------------------
		// DRAW NORMALS!!
	if (m_style_flag & FLAG_SHOW_NORMALS) {
		for (int i = 0; i < mynormals.size(); i++) {
			mynormals[i].draw();
		}
	}
	//------------------------------------- CURVATURES -----------------------------------------
	if (DRAW_CURVATURES && !REGION_GROWING) {
		for (int k = 0; k < mycur.size(); k++) {
			mycur[k].draw();
		}
	}

	// ---------------------------------- REGION GROWING ----------------------------------------
	if (DRAW_CURVATURES && REGION_GROWING && !REMOVE_GROUND) {
		for (int i = 0; i < region_growing.size(); i++) {
			region_growing[i].draw();
		}
	}
	// ---------------------------------- SHOW OBJECTS ----------------------------------------
	if (DRAW_CURVATURES && REGION_GROWING && REMOVE_GROUND ) {
		for (int i = 0; i < m_obj.size(); i++) {
			m_obj[i].draw();
		}
	}

	// ---------------------------------- TRIANGULATION ------------------------------------------
		 //if (TRIANGULATION) m_object.draw(m_obj_col, SOLID);
	if (TRIANGULATION1) m_object.draw(Colour::black, WIRE);
	if (DRAW_CURVATURES && REGION_GROWING && REMOVE_GROUND && TRIANGULATION2) m_object.draw(Colour::black, WIRE);
}

void pca(vector<vec>& vertices, vec& center, vec& dir)
{
	const int count = vertices.size();

	float w0 = 0;
	float x0 = 0, y0 = 0, z0 = 0;
	float x2 = 0, y2 = 0, z2 = 0, xy = 0, yz = 0, xz = 0;
	float dx2, dy2, dz2, dxy, dxz, dyz;
	float det[9];

	for (int i = 0; i < count; i++)
	{
		float x = vertices[i].x;
		float y = vertices[i].y;
		float z = vertices[i].z;

		x2 += x * x;
		xy += x * y;
		xz += x * z;
		y2 += y * y;
		yz += y * z;
		z2 += z * z;
		x0 += x;
		y0 += y;
		z0 += z;
	}
	w0 = (float)count;

	x2 /= w0;
	xy /= w0;
	xz /= w0;
	y2 /= w0;
	yz /= w0;
	z2 /= w0;

	x0 /= w0;
	y0 /= w0;
	z0 /= w0;

	dx2 = x2 - x0 * x0;
	dxy = xy - x0 * y0;
	dxz = xz - x0 * z0;
	dy2 = y2 - y0 * y0;
	dyz = yz - y0 * z0;
	dz2 = z2 - z0 * z0;

	det[0] = dz2 + dy2;
	det[1] = -dxy;
	det[2] = -dxz;
	det[3] = det[1];
	det[4] = dx2 + dz2;
	det[5] = -dyz;
	det[6] = det[2];
	det[7] = det[5];
	det[8] = dy2 + dx2;

	/* Searching for a eigenvector of det corresponding to the minimal eigenvalue */
	gte::SymmetricEigensolver3x3<float> solver;
	std::array<float, 3> eval;
	std::array<std::array<float, 3>, 3> evec;
	solver(det[0], det[1], det[2], det[4], det[5], det[8], true, 1, eval, evec);

	center.x = x0;
	center.y = y0;
	center.z = z0;

	float max = std::abs(eval[0]);
	int index = 0;
	if (std::abs(eval[1]) > max) { max = std::abs(eval[1]);  index = 1; }
	if (std::abs(eval[2]) > max) { max = std::abs(eval[2]); index = 2; }
	dir.x = evec[index][0];
	dir.y = evec[index][1];
	dir.z = evec[index][2];


	//cout <<"i=" <<index << endl;
	/*
	vec a;
	vec b;

	a.x = evec[0][0];
	a.y = evec[0][1];
	a.z = evec[0][2];

	b.x = evec[1][0];
	b.y = evec[1][1];
	b.z = evec[1][2];



	float temp1;
	float temp2;
	float temp3;
	dir.x = (a.y * b.z) - (b.y*a.z);
	dir.y = (a.x * b.z) - (a.z * b.x);
	dir.z = (a.x * b.y) - (a.y * b.x);
	*/


	mydirs_x.push_back(dir.x);

}

float find_Curvature(vector<vec>& vertices, vec& center, vec& dir) {
	const int count = vertices.size();

	float w0 = 0;
	float x0 = 0, y0 = 0, z0 = 0;
	float x2 = 0, y2 = 0, z2 = 0, xy = 0, yz = 0, xz = 0;
	float dx2, dy2, dz2, dxy, dxz, dyz;
	float det[9];

	for (int i = 0; i < count; i++)
	{
		float x = vertices[i].x;
		float y = vertices[i].y;
		float z = vertices[i].z;

		x2 += x * x;
		xy += x * y;
		xz += x * z;
		y2 += y * y;
		yz += y * z;
		z2 += z * z;
		x0 += x;
		y0 += y;
		z0 += z;
	}
	w0 = (float)count;

	x2 /= w0;
	xy /= w0;
	xz /= w0;
	y2 /= w0;
	yz /= w0;
	z2 /= w0;

	x0 /= w0;
	y0 /= w0;
	z0 /= w0;

	dx2 = x2 - x0 * x0;
	dxy = xy - x0 * y0;
	dxz = xz - x0 * z0;
	dy2 = y2 - y0 * y0;
	dyz = yz - y0 * z0;
	dz2 = z2 - z0 * z0;

	det[0] = dz2 + dy2;
	det[1] = -dxy;
	det[2] = -dxz;
	det[3] = det[1];
	det[4] = dx2 + dz2;
	det[5] = -dyz;
	det[6] = det[2];
	det[7] = det[5];
	det[8] = dy2 + dx2;

	/* Searching for a eigenvector of det corresponding to the minimal eigenvalue */
	gte::SymmetricEigensolver3x3<float> solver;
	std::array<float, 3> eval;
	std::array<std::array<float, 3>, 3> evec;
	solver(det[0], det[1], det[2], det[4], det[5], det[8], true, 1, eval, evec);

	center.x = x0;
	center.y = y0;
	center.z = z0;
	float min = eval[0];
	if (eval[1] < min) { min = eval[1]; }
	if (eval[2] < min) { min = eval[2]; }
	//CURVATURE
	m_curvature = eval[0] / (eval[0] + eval[1] + eval[2]);

	my_curvatures.push_back(m_curvature);


	return m_curvature;
}

void new_pca(vector<vec>& vertices, vec& center, float& mean, float& gaussian, vec& principal)
{
	const int count = vertices.size();

	float w0 = 0;
	float x0 = 0, y0 = 0, z0 = 0;
	float x2 = 0, y2 = 0, z2 = 0, xy = 0, yz = 0, xz = 0;
	float dx2, dy2, dz2, dxy, dxz, dyz;
	float det[9];

	for (int i = 0; i < count; i++)
	{
		float x = vertices[i].x;
		float y = vertices[i].y;
		float z = vertices[i].z;

		x2 += x * x;
		xy += x * y;
		xz += x * z;
		y2 += y * y;
		yz += y * z;
		z2 += z * z;
		x0 += x;
		y0 += y;
		z0 += z;
	}
	w0 = (float)count;

	x2 /= w0;
	xy /= w0;
	xz /= w0;
	y2 /= w0;
	yz /= w0;
	z2 /= w0;

	x0 /= w0;
	y0 /= w0;
	z0 /= w0;

	dx2 = x2 - x0 * x0;
	dxy = xy - x0 * y0;
	dxz = xz - x0 * z0;
	dy2 = y2 - y0 * y0;
	dyz = yz - y0 * z0;
	dz2 = z2 - z0 * z0;

	det[0] = dz2 + dy2;
	det[1] = -dxy;
	det[2] = -dxz;
	det[3] = det[1];
	det[4] = dx2 + dz2;
	det[5] = -dyz;
	det[6] = det[2];
	det[7] = det[5];
	det[8] = dy2 + dx2;


	gte::SymmetricEigensolver3x3<float> solver;
	std::array<float, 3> eval;
	std::array<std::array<float, 3>, 3> evec;
	solver(det[0], det[1], det[2], det[4], det[5], det[8], true, 1, eval, evec);

	center.x = x0;
	center.y = y0;
	center.z = z0;


	principal.x = evec[2][0];
	principal.y = evec[2][1];
	principal.z = evec[2][2];

	//mean
	mean = (eval[0] + eval[2]) / 2;


	//Gaussian
	gaussian = eval[0] * eval[2];

}

void Mesh3DScene::Task_Draw_PCA(vec& center, vec& dir)
{
	//!//////////////////////////////////////////////////////////////////////////////////
	//! TASK:
	//!
	//!  - Apeikoniste to kentro mazas kai ton Principal Axis tou PCA.
	//!    Gia tin apeikonisi, xreiazeste ena simeio kai mia eytheia.
	//!
	//! HINTS:
	//!  - Auti i synartisi kaleitai mesa apo tin `Mesh3DScene::draw()`.
	//!    Ara, mporeite na kalesete amesa tis metodous draw ton diaforwn antikeimenwn
	//!
	//!//////////////////////////////////////////////////////////////////////////////////
	//vec start = dir;
	vec end = dir;
	//start *= -0.2;
	//end *= 0.2;
	//start += center;
	end += center;


	Colour base_colour;
	//base_colour.r = 255.0 * (dir.x - (mynewarray[0].normal.x) / ((mynewarray[mynewarray.size()-1].normal.x) -(mynewarray[0].normal.x)));
	base_colour.r = 255.0 * (dir.x - mydirs_x[0]) / (mydirs_x[mydirs_x.size() - 1] - mydirs_x[0]);
	base_colour.g = 0.0;
	base_colour.b = 255.0;

	LineSeg3D line(center.x, center.y, center.z, end.x, end.y, end.z, base_colour);
	mynormals.push_back(line);
	//line.draw();


}

void Mesh3DScene::draw_Curvature(vec& cur, float& curvature) {
	Colour base_colour;
	base_colour.r = 255.0 * (curvature - my_curvatures[0]) / my_curvatures[my_curvatures.size() - 1];
	base_colour.g = 0.0;
	base_colour.b = 255.0;

	Point3D test(cur.x, cur.y, cur.z, base_colour);
	mycur.push_back(test);
}

/*
void Task_Draw_new_PCA(vec& center, vec& princ)
{
	//!//////////////////////////////////////////////////////////////////////////////////
	//! TASK:
	//!
	//!  - Apeikoniste to kentro mazas kai ton Principal Axis tou PCA.
	//!    Gia tin apeikonisi, xreiazeste ena simeio kai mia eytheia.
	//!
	//! HINTS:
	//!  - Auti i synartisi kaleitai mesa apo tin `Mesh3DScene::draw()`.
	//!    Ara, mporeite na kalesete amesa tis metodous draw ton diaforwn antikeimenwn
	//!
	//!//////////////////////////////////////////////////////////////////////////////////
	vec start = princ;
	vec end = princ;
	start *= -0.05;
	end *= 0.05;
	start += center;
	end += center;

	Colour base_colour;
	base_colour.r = 10.0 + 128.0 * princ.x;
	base_colour.g = 10.0 + 128.0 * princ.y;
	base_colour.b = 10.0 + 128.0 * princ.z;

	LineSeg3D line(start.x, start.y, start.z, end.x, end.y, end.z, base_colour);
	mycurvs.push_back(line);
	//line.draw();

}
 */

void Region_Growing(vector<Point_Of_Interest> mynewarray, vector<std::vector<Point_Of_Interest> >  R, vector<vec> sorted_point_cloud, KDTree* m_KDTree, Sphere3D m_sphere_new_1) {

	for (int i = 0; i < mynewarray.size(); i++) {
		sorted_point_cloud.push_back(mynewarray[i].m_vertice);
	}

	delete m_KDTree;
	m_KDTree = new KDTree(sorted_point_cloud);
	//m_tree_invalidation_sec = -1;

	//m_anim.setTime(0);

	//if (TROPOS == 6) {
	float max = -5;
	do {
		std::vector<Point_Of_Interest> Sc;
		std::vector<Point_Of_Interest> Rc;
		Sc.push_back(mynewarray[0]);
		Rc.push_back(mynewarray[0]);
		mynewarray.erase(mynewarray.begin());
		sorted_point_cloud.erase(sorted_point_cloud.begin());

		while (Sc.size() != 0) {
			
			vvr::Sphere3D sphere_moved_new_1(m_sphere_new_1);
			sphere_moved_new_1.x = Sc[0].m_vertice.x;
			sphere_moved_new_1.y = Sc[0].m_vertice.y;
			sphere_moved_new_1.z = Sc[0].m_vertice.z;
			Sphere sphere_new(Sc[0].m_vertice, sphere_moved_new_1.rad);
			std::vector<vec> Neighbors;
			Task_Neighboors_InSphere(sphere_new, m_KDTree->root(), Neighbors);

			for (int j = 0; j < Neighbors.size(); j++) {
				vec somepoint = Neighbors[j];
				auto it = std::find_if(mynewarray.begin(), mynewarray.end(), [&somepoint](const Point_Of_Interest& p) {
					return somepoint.x == p.m_vertice.x && somepoint.y == p.m_vertice.y && somepoint.z == p.m_vertice.z; });
				if (it != mynewarray.end()) {
					int index = it - mynewarray.begin();
					float angle = std::acos((Sc[0].normal.Dot(mynewarray[index].normal)) / (Sc[0].normal.Length() * mynewarray[index].normal.Length()));
					if (angle > max) {
						max = angle;
					}
					if (angle < 1.2) {
						Rc.emplace_back(mynewarray[index]);
						Point_Of_Interest temp = mynewarray[index];
						float temp2 = mynewarray[index].curvature;
						mynewarray.erase(mynewarray.begin() + index);
						sorted_point_cloud.erase(sorted_point_cloud.begin() + index);

						if (temp2 < 0.085) {
							Sc.emplace_back(temp);
						}

					}
				}

			}

			Sc.erase(Sc.begin());
		}

		R.push_back(Rc);
	} while (!mynewarray.empty() || !sorted_point_cloud.empty());

}

int main(int argc, char* argv[])
{
	try {
		return vvr::mainLoop(argc, argv, new Mesh3DScene);
	}
	catch (std::string exc) {
		cerr << exc << endl;
		return 1;
	}
	catch (...)
	{
		cerr << "Unknown exception" << endl;
		return 1;
	}
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
KDTree::KDTree(VecArray& pts)
	: pts(pts)
{
	const float t = vvr::getSeconds();                                    //xronos
	m_root = new KDNode();
	m_depth = makeNode(m_root, pts, 0);
	const float KDTree_construction_time = vvr::getSeconds() - t;         //xronos
	echo(KDTree_construction_time);
	echo(m_depth);
}

KDTree::~KDTree()
{
	const float t = vvr::getSeconds();                                    //xronos
	delete m_root;
	const float KDTree_destruction_time = vvr::getSeconds() - t;          //xronos
	echo(KDTree_destruction_time);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int KDTree::makeNode(KDNode* node, VecArray& pts, const int level)
{
	//! Sort along the appropriate axis, find median point and split.
	const int axis = level % DIMENSIONS;
	std::sort(pts.begin(), pts.end(), VecComparator(axis));
	const int i_median = pts.size() / 2;

	//! Set node members
	node->level = level;
	node->axis = axis;
	node->split_point = pts[i_median];
	node->aabb.SetFrom(&pts[0], pts.size());

	//! Continue recursively or stop.
	if (pts.size() <= 1)                                               //afh ths sxesh thn vazoume gia na stamathsei h anadromh
	{
		return level;
	}
	else
	{
		int level_left = 0;
		int level_right = 0;
		VecArray pts_left(pts.begin(), pts.begin() + i_median);
		VecArray pts_right(pts.begin() + i_median + 1, pts.end());

		if (!pts_left.empty())
		{
			node->child_left = new KDNode();
			level_left = makeNode(node->child_left, pts_left, level + 1);

		}
		if (!pts_right.empty())
		{
			node->child_right = new KDNode();
			level_right = makeNode(node->child_right, pts_right, level + 1);
		}

		int max_level = std::max(level_left, level_right);
		return max_level;
	}
}

void KDTree::getNodesOfLevel(KDNode* node, std::vector<KDNode*>& nodes, int level)
{
	if (!level)
	{
		nodes.push_back(node);
	}
	else
	{
		if (node->child_left) getNodesOfLevel(node->child_left, nodes, level - 1);
		if (node->child_right) getNodesOfLevel(node->child_right, nodes, level - 1);
	}
}

std::vector<KDNode*> KDTree::getNodesOfLevel(const int level)
{
	std::vector<KDNode*> nodes;
	if (!m_root) return nodes;
	getNodesOfLevel(m_root, nodes, level);
	return nodes;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Task_01_FindPtsOfNode(const KDNode* root, VecArray& pts)
{
	pts.push_back(root->split_point);
	if (root->child_left) {
		Task_01_FindPtsOfNode(root->child_left, pts);
	}

	if (root->child_right) {
		Task_01_FindPtsOfNode(root->child_right, pts);
	}
}

void Task_Neighboors_InSphere(const Sphere& sphere, const KDNode* root, VecArray& pts)

{
	if (!root) return;
	// Distance  between two points
	const double d = sphere.pos.Distance(root->split_point);
	const double d_split = root->split_point.ptr()[root->axis] - sphere.pos.ptr()[root->axis];
	const bool right_of_split = d_split <= 0;

	if (d <= sphere.r && d != 0) {
		pts.push_back(root->split_point);
	}

	Task_Neighboors_InSphere(sphere, right_of_split ? root->child_right : root->child_left, pts);

	if (d_split > sphere.r)return;

	Task_Neighboors_InSphere(sphere, right_of_split ? root->child_left : root->child_right, pts);
}

void Task_Find_NearestK(const int k, const vec& test_pt, const KDNode* root, const KDNode** knn, float* best_dist)
{
	if (!root) return;
	// Distance  between two points
	const float d = test_pt.Distance(root->split_point);
	const double d_split = root->split_point.ptr()[root->axis] - test_pt.ptr()[root->axis];
	const bool right_of_split = d_split <= 0;

	//(eimai sthn riza kai thewrw pws ksekinaw apo to x, sugkrinw x<=x1) 
	if (k == 0 && (knn[k] == NULL || d < *best_dist) && d != 0) {
		*best_dist = d;
		knn[k] = root;
	}
	int count = 0;
	if (k >= 1 && (knn[k] == NULL || d < *best_dist) && d != 0) {
		for (int i = 0; i <= k - 1; i++) {
			if (root != knn[i]) {
				count++;
			}
		}
		if (count == k) {
			*best_dist = d;
			knn[k] = root;
		}
	}

	//searching
	Task_Find_NearestK(k, test_pt, right_of_split ? root->child_right : root->child_left, knn, best_dist);
	//pruning diagrafh
	if (SQUARE(d_split) >= *best_dist)return;
	//searching
	Task_Find_NearestK(k, test_pt, right_of_split ? root->child_left : root->child_right, knn, best_dist);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void Triangulate_Objects(std::vector<vec> object, int m_kn, KDTree* m_object_KDTree, int m_current_o_tree_level, Mesh& model) {

	//m_o_tree_invalidation_sec = -1;

	//m_anim.setTime(0);
	while (!object.empty()) {
		//for (int w = 0; w < object.size(); w++) {
		m_current_o_tree_level = 0;
		delete m_object_KDTree;
		m_object_KDTree = new KDTree(object);

		float dist;
		const KDNode** nearests = new const KDNode * [m_kn];
		memset(nearests, NULL, m_kn * sizeof(KDNode*));

		for (int i = 0; i < m_kn; i++) {
			Task_Find_NearestK(i, object[0], m_object_KDTree->root(), nearests, &dist);
		}

		//for (int i = 0; i < m_kn; i++) {
		//if (!nearests[i]) continue;
		vec nn1 = nearests[0]->split_point;
		vec nn2 = nearests[1]->split_point;

		model.getVertices().push_back(object[0]);
		model.getVertices().push_back(nn1);
		model.getVertices().push_back(nn2);

		int index = model.getVertices().size() - 3;
		model.getTriangles().push_back(vvr::Triangle(&model.getVertices(), index, index + 1, index + 2));


		delete[] nearests;

		model.update();
		object.erase(object.begin());
		//w--;

		//}
	}
}

bool Is_On_Right(vec q, vec p, vec t) {
	double val1 = (q.y - p.y) * (t.z - q.z) - (t.y - q.y) * (q.z - p.z);
	double val2 = (q.x - p.x) * (t.z - q.z) - (t.x - q.x) * (q.z - p.z);
	double val3 = (q.x - p.x) * (t.y - q.y) - (t.x - q.x) * (q.y - p.y);
	double val = val1 - val2 + val3;
	//cout << val << endl;
	if (val > 0) {
		return true;
	}
	else return false;

}

vector<vec> convex_hull(std::vector<vec> input, std::vector<vec> convex_hull_points) {
	int sum = 0;
	for (int i = 0; i < input.size(); i++) {
		for (int j = i + 1; j < input.size(); j++) {
			for (int t = 0; t < input.size(); t++) {
				if (Is_On_Right(input[i], input[j], input[t]) == true && input[t].x != input[i].x && input[t].y != input[i].y && input[t].z != input[i].z
					&& input[t].x != input[j].x && input[t].y != input[j].y && input[t].z != input[j].z) {
					sum = sum + 1;
				}
				if (sum >= input.size() - 2) {
					convex_hull_points.push_back(input[i]);
					convex_hull_points.push_back(input[j]);
				}
			}
			sum = 0;
		}
	}
	return convex_hull_points;
}

void Add_To_Mesh(std::vector<vec> c_h_points) {
	for (int i = 0; i < c_h_points.size() - 2; i++) {
		m_object.getVertices().push_back(c_h_points[0]);
		m_object.getVertices().push_back(c_h_points[i + 1]);
		m_object.getVertices().push_back(c_h_points[i + 2]);

		int index = m_object.getVertices().size() - 3;
		m_object.getTriangles().push_back(vvr::Triangle(&m_object.getVertices(), index, index + 1, index + 2));
	}
	//m_object.update();
}

