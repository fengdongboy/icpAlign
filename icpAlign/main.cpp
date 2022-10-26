
#include <string>

#include "Align.h"
#include "TriMesh.h"

#define MESH_NUM 84
#define MESH_PATH "F:/work/F1_Scanner/Fox Scan/Fox Scan/esslam_save/2022-10-25-09-15-31"

/// fx fy cx cy 1015.47 1015.47 32.6436 323.577
#define fxParams 1015.47, 1015.47, 32.6436, 323.577

void load(trimesh::TriMesh& mesh, std::fstream& stream);
int main()
{
	Align* a = new Align;
	a->setParams(fxParams);
	a->setup();
	a->setParams(fxParams);

	
	for (int i = 0; i < MESH_NUM; i++)
	{
		std::string str = MESH_PATH; str+="/" + std::to_string(i) + ".cloud";
		std::fstream fin(str.c_str(), std::ios::in | std::ios::binary);
		trimesh::TriMesh * m = new trimesh::TriMesh;
		load(*m, fin);
		m->need_bbox();
		TriMeshPtr mesh_ptr(m);
		a->ProcessOneFrame(mesh_ptr);
	}

	trimesh::TriMesh& mesh = a->getFushMesh();
	mesh.write("total.ply");

	delete a;
	return 0;
}

void load(trimesh::TriMesh& mesh, std::fstream& stream)
{
	int ver_num = 0;
	stream.read((char*)&ver_num, sizeof(int));
	if (ver_num)
	{
		mesh.vertices.resize(ver_num);
		mesh.normals.resize(ver_num);
		stream.read((char*)&mesh.vertices[0], 3 * sizeof(float) * ver_num);
		stream.read((char*)&mesh.normals[0], 3 * sizeof(float) * ver_num);
	}
	mesh.colors.resize(ver_num);
	int grid_width = 0;
	int grid_height = 0;
	stream.read((char*)&grid_width, sizeof(int));
	stream.read((char*)&grid_height, sizeof(int));
	mesh.grid_width = grid_width;
	mesh.grid_height = grid_height;
	if (grid_width > 0 && grid_height > 0)
	{
		mesh.grid.resize(grid_width * grid_height);
		stream.read((char*)&mesh.grid[0], grid_width * grid_height * sizeof(int));
	}
}