

#ifndef mesh_functions_hpp
#define mesh_functions_hpp

#include <iostream>
#include <limits>
#include <random>
#include "starter_defs.h"
#include <CGAL/IO/Color.h>

using namespace std ;

CGAL::Color rand_color(std::uniform_int_distribution<> &distr, std::mt19937 &gen) ;

vector<Mesh::Vertex_index> vertex_neighbors (Mesh::Vertex_index v, const Mesh &m);

CGAL::Color color_ramp(double vmin, double vmax, double mean, double stdev, double v, CGAL::Color col1, CGAL::Color col2);

vector<Mesh::Vertex_index> vneigh (Mesh::Vertex_index v, Mesh &m);

int add_deg_to_vertices(Mesh &m);

void create_color_card_per_vertice_degree(Mesh &m);

double gaussian_curvature(Mesh::Vertex_index v, const Mesh &m);

void gaussian_curv(Mesh &m, Mesh::Property_map<vertex_descriptor,double> &Kcourb);

#endif /* mesh_functions_hpp */
