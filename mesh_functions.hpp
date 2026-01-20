

#ifndef mesh_functions_hpp
#define mesh_functions_hpp

#include <iostream>
#include <limits>
#include <random>
#include "starter_defs.h"
#include <CGAL/IO/Color.h>
#include <cmath>
#include <vector>
#include <algorithm> // pour std::max etc.

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std ;

CGAL::Color rand_color(std::uniform_int_distribution<> &distr, std::mt19937 &gen) ;

CGAL::Color color_ramp(double vmin, double vmax, double mean, double stdev, double v, CGAL::Color col1, CGAL::Color col2);


// Partie une du TP 1

// Voisins d'un sommet
std::vector<Mesh::Vertex_index> vneightbors(Mesh::Vertex_index v, const Mesh &m);

// Calcul des degrés
void compute_vertex_degrees(Mesh &m);

// Coloration selon le degré
void color_mesh_by_degree(Mesh &m);


// Partie deux du TP 1

void gaussian_cruve(Mesh &m, Mesh::Property_map<Mesh::Vertex_index, double> &Kcourb);

void color_mesh_by_curvature(Mesh &m);

#endif /* mesh_functions_hpp */
