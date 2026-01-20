//
//  test1.cpp
//  
//
//  Created by Bac Alexandra on 11/02/2022.
//

#include <iostream>
#include <limits>
#include <random>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/IO/polygon_mesh_io.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Kernel/global_functions.h>
#include "distrib.hpp"
#include "starter_defs.h"
#include "mesh_functions.hpp"

using namespace std ;

int main(int argc, char** argv)
{
	std::random_device rd; // obtain a random number from hardware
	std::mt19937 gen(rd()); // seed the generator
	std::uniform_int_distribution<> distr(0, 255); // define the range

	Mesh m;
	string fname ;
	if (argc != 2)
	{
		cout << "Wrong number of arguments: test1 filename" << endl ;
		return EXIT_FAILURE;
	}
	else
	{
		fname = argv[1] ;
	}
	
	if(!CGAL::IO::read_polygon_mesh(fname, m))
	{
		std::cerr << "Invalid input file." << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Mesh loaded: " << m.number_of_vertices() << " sommets" << std::endl;

	// Calcul les degrés
	compute_vertex_degrees(m);

	// Coloration selon les degrés
	//color_mesh_by_degree(m);

	// Coloration selon les courbures gaussiennes
	color_mesh_by_curvature(m);

	// Sauvegarde du maillage coloré
	std::string out_name = "test_colored.off";
	if(!CGAL::IO::write_polygon_mesh(out_name, m, CGAL::parameters::vertex_color_map(m.property_map<Mesh::Vertex_index, CGAL::Color>("v:color").first))) {
		std::cerr << "Erreur écriture fichier." << std::endl;
		return 1;
	}

	std::cout << "Fichier genere : " << out_name << std::endl;


	return 0;
}

