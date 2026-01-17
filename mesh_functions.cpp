//
//  mesh_functions.cpp
//  test1
//
//  Created by Bac Alexandra on 17/02/2022.
//

#include <algorithm>
#include "mesh_functions.hpp"


CGAL::Color rand_color(std::uniform_int_distribution<> &distr, std::mt19937 &gen)
{
	CGAL::Color c ;
	c.red() = distr(gen) ;
	c.green() = distr(gen) ;
	c.blue() = distr(gen) ;
	return c ;
}

CGAL::Color color_ramp(double vmin, double vmax, double mean, double stdev, double v, CGAL::Color col1, CGAL::Color col2)
{
    stdev = stdev/2 ;
    if (vmin < mean-stdev)
        vmin = mean-stdev ;
    if (vmax > mean+stdev)
        vmax = mean+stdev ;
    if (v < vmin)
        v = vmin ;
    else if (v > vmax)
        v = vmax ;
    double per ;
    CGAL::Color c ;
    CGAL::Color col_first, col_second ;
    if (v <= mean)
    {
        per = (v-vmin)/(mean-vmin) ;
        col_first = col1 ;
        col_second = CGAL::white() ;
    }
    else
    {
        per = (v-mean)/(vmax-mean) ;
        col_first = CGAL::white() ;
        col_second = col2 ;
    }
    c.red() = floor(col_first.red() + per*(col_second.red()-col_first.red())) ;
    c.green() = floor(col_first.green() + per*(col_second.green()-col_first.green())) ;
    c.blue() = floor(col_first.blue() + per*(col_second.blue()-col_first.blue())) ;
    return c ;
}

vector<Mesh::Vertex_index> vneigh (Mesh::Vertex_index v, Mesh &m) {
    vector<Mesh::Vertex_index> neighbors ;
    Mesh::Halfedge_index h = m.halfedge(v) ;
    Mesh::Halfedge_index start = h ;
    do {
        Mesh::Vertex_index vn = m.target(h) ;
        neighbors.push_back(vn) ;
        h = m.next(m.opposite(h)) ;
    } while (h != start) ;
    return neighbors ;
}

int add_deg_to_vertices(Mesh &m) {
    Mesh::Property_map<vertex_descriptor, int> deg ;
    bool created_deg ;
    boost::tie(deg, created_deg) = m.add_property_map<vertex_descriptor, int>("v:deg", 0) ;
    assert(created_deg) ;

    int max_deg = 0 ;
    for (Mesh::Vertex_index vi : m.vertices()) {
        deg[vi] = static_cast<int>(vneigh(vi, m).size()) ;
        if (deg [vi] > max_deg) {
            max_deg = deg[vi] ;
        }
    }

    std::cout << "Max degree: " << max_deg << std::endl ;

    return max_deg;
}

void create_color_card_per_vertice_degree(Mesh &m) {
    Mesh::Property_map<vertex_descriptor, CGAL::Color> color ;
    bool created_col ;
    boost::tie(color, created_col) = m.add_property_map<vertex_descriptor, CGAL::Color>("v:color", CGAL::Color()) ;
    assert(created_col) ;

    int max_deg = add_deg_to_vertices(m);

    for (Mesh::Vertex_index vi : m.vertices()) {
        int deg = static_cast<int>(m.degree(vi)) ;
        color[vi] = color_ramp(0, max_deg, max_deg / 2.0, max_deg / 2.0, deg, CGAL::blue(), CGAL::red()) ;
    }
}


double gaussian_curvature(Mesh::Vertex_index v, const Mesh &m) {
    double K = 2 * CGAL_PI ;
    Mesh::Halfedge_index h = m.halfedge(v) ;
    Mesh::Halfedge_index start = h ;
    do {
        Mesh::Face_index f = m.face(h) ;
        if (f != Mesh::null_face()) {
            K -= CGAL::approximate_angle(m.point(m.target(h)),
                                         m.point(m.target(m.next(h))),
                                         m.point(m.target(m.prev(h)))) ;
        }
        h = m.next(m.opposite(h)) ;
    } while (h != start) ;
    return K ;
}

void gaussian_curv(Mesh &m, Mesh::Property_map<vertex_descriptor,double> &Kcourb) {
    for (Mesh::Vertex_index v : m.vertices()) {
        Kcourb[v] = gaussian_curvature(v, m) ;
    }
}