//
//  mesh_functions.cpp
//  test1
//
//  Created by Bac Alexandra on 17/02/2022.
//

#include <algorithm>
#include "mesh_functions.hpp"
#include "distrib.hpp"


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

std::vector<Mesh::Vertex_index> vneighbors(Mesh::Vertex_index v, const Mesh &m) {
    std::vector<Mesh::Vertex_index> neighbors;

    // On récupère une demi arrête SORTANTE du sommet v
    Mesh::halfedge_index h = m.halfedge(v);

    // Cas sommet isolé pas de voisin
    if (h == Mesh::null_halfedge()) { return neighbors; }

    // On circule autour du sommet sans utiliser de circulateurs
    Mesh::Halfedge_index start = h;
    do {
        // Le voisin est la cible de la demi-arête sortante actuelle
        Mesh::Vertex_index neighbor = m.target(h);
        neighbors.push_back(neighbor);

        // Rotation autour du sommet source
        // opposite(h) nous ramène vers v (demi-arête entrante)
        // next(...) nous donne la suivante dans la face (prochaine demi-arête sortante)
        h = m.next(m.opposite(h));

    } while (h != start); // Tant qu'on n'est pas revenu au point de départ

    return neighbors;
}

void compute_vertex_degrees(Mesh &m) {
    // Création ou récupération de la propriété "v:deg"
    // <Key, Value> -> <Vertex_index, int>
    Mesh::Property_map<Mesh::Vertex_index, int> degree_prop;
    bool created;

    // m.add_property_map<type>(nom, valeur_par_defaut)
    boost::tie(degree_prop, created) = m.add_property_map<Mesh::Vertex_index, int>("v:deg", 0);

    // On parcourt tous les sommets du maillage
    for (auto v : m.vertices()) {
        // Utilisation de la fonction précédente vneighbors(...)
        std::vector<Mesh::Vertex_index> neighbors = vneighbors(v, m);

        // On stocke le degré dans la propriété
        degree_prop[v] = neighbors.size();
    }

    std::cout << "Calcul des degrés terminé." << std::endl;
}

void color_mesh_by_degree(Mesh &m) {
    // Récupérer la propriété des degrés (Doit exister)
    Mesh::Property_map<Mesh::Vertex_index, int> degree_prop;
    bool exists;
    boost::tie(degree_prop, exists) = m.property_map<Mesh::Vertex_index, int>("v:deg");

    if (!exists) {
        std::cerr << "La propriété des degrés n'existe pas. Veuillez la calculer d'abord." << std::endl;
        return;
    }

    // Calcul des statistiques (min, max, mean, ecart-type)
    double sum = 0.0;
    double sum_sq = 0.0; // Somme des carrés pour la variance
    int count = 0;
    int min_val = 10000; // Valeur arbitrairement grande
    int max_val = 0;

    for (auto v : m.vertices()) {
        double d = (double)degree_prop[v];
        sum += d;
        sum_sq += d*d;
        if (d < min_val) min_val = (int)d;
        if (d > max_val) max_val = (int)d;
        count++;
    }

    if (count == 0) return;

    double mean = sum/count;
    double variance = (sum_sq / count) - (mean * mean);
    double stdev = std::sqrt(variance); // Ecart-type standard

    std::cout << "Stats Degrees -> Moyenne : " << mean << ", Min : " << min_val << ", Max : " << max_val << ", Ecart-type : " << stdev << std::endl;

    // Création de la propriété couleur
    Mesh::Property_map<Mesh::Vertex_index, CGAL::Color> color_prop;
    bool created;
    boost::tie(color_prop, created) = m.add_property_map<Mesh::Vertex_index,CGAL::Color>("v:color");

    // Définition des couleurs extrème (Bleu pour faible degré, Rouge pour fort degré)
    CGAL::Color low_color = CGAL::blue();
    CGAL::Color high_color = CGAL::red();

    // Application de color_ramp sur chaque sommet
    for (auto v : m.vertices()) {
        double val = (double)degree_prop[v];

        // On appelle la fonction color_ramp donnée
        // Elle interpole bleu blanc et rouge
        color_prop[v] = color_ramp((double)min_val, (double)max_val, mean, stdev, val, low_color, high_color);
    }
}

void gaussian_curv(Mesh &m, Mesh::Property_map<Mesh::Vertex_index, double> &Kcourb) {
    for (auto v : m.vertices()) {
        // Pour les sommets de bord, la formule est différente et souvent on met 0 ou on ignore
        if (m.is_border(v)) {
            Kcourb[v] = 0.0;
            continue;
        }

        double sum_angles = 0.0;
        double sum_areas = 0.0;

        K::Point_3 p = m.point(v);

        // On circule autour du sommet v
        // Pour calculer l'angle et l'aire, il nous faut le point voisin actuel (pi) et le suivant (pj)
        Mesh::Halfedge_index h = m.halfedge(v);
        Mesh::Halfedge_index start = h;

        // Sécurité pour sommmets isolés
        if (h == Mesh::null_halfedge()) {
            Kcourb[v] = 0.0;
            continue;
        }

        do {
            // h part de v vers Pi
            Mesh::Vertex_index pi = m.target(h);

            // h_next part de pi vers pj (dans le triangle)
            // ATTENTION : pour avoir l'angle en v, il nous faut les vecteurs v->pi et v->pj
            // pj est le sommet d'après dans le sens trigo autour de v
            Mesh::Halfedge_index h_prev = m.opposite(m.next(h)); // Astuce pour récupérer l'arête précédente entrant dans v
            // Mais plus simple regardons le triangle formé par h

            // Stratégie simple :
            // h va de V -> Pi
            // m.next(h) va de P1 -> P2
            // m.next(m.next(h)) va de P2 -> V
            // Donc les voisins sont target(h) et target(next(h)) ? NON next(h) tourne dans la face.

            // REVENONS A la base du "One Ring"$
            // Voisin 1 (A) = m.target(h)
            // Voisin 2 (B) = m.target(m.next(m.opposite(h))) -> non ça c'est le voisin suivant dans le one-ring

            Mesh::Vertex_index v_neighbor = m.target(h);

            // Pour avoir le triangle (v, neighbor, next_neighbor), il faut l'arête qui suit h dans la FACE
            Mesh::Halfedge_index h_next_in_face = m.next(h);
            Mesh::Vertex_index v_next_in_face = m.target(h_next_in_face);

            // Points géométriques
            K::Point_3 p_neighbor = m.point(v_neighbor); // Pi
            K::Point_3 p_next = m.point(v_next_in_face); // Pi+1 dans le triangle

            // Vecteurs
            K::Vector_3 u = p_neighbor - p; // V -> Pi
            K::Vector_3 w = p_next - p; // V -> Pi+1

            // Calcul de l'air du triangle (Ti)
            // Aire = 0.5 * || u ^ w ||
            // CGAL::cross_product(u, w) calcule le produit vectoriel
            double area_triangle = 0.5 * std::sqrt(CGAL::cross_product(u,w).squared_length());
            sum_areas += area_triangle;

            // Calcul de l'angle alpha_i en V entre u et w
            // u . w = |u| * |w| * cos(theta) => theta = arccos( (u.w) / (|u|*|w|) )
            double dot = u * w; // Produit scalaire
            double len_u = std::sqrt(u.squared_length());
            double len_w = std::sqrt(w.squared_length());

            // Protection division par zéro
            if (len_u > 1e-10 && len_w > 1e-10) {
                double cos_val = dot / (len_u * len_w);
                // Clamp pour éviter les erreurs numériques hors de [-1, 1]
                if (cos_val > 1.0) cos_val = 1.0;
                if (cos_val < -1.0) cos_val = -1.0;
                sum_angles += std::acos(cos_val);
            }

            // Passer à l'arête suivante autour du sommet V
            h = m.next(m.opposite(h));
        } while (h != start);

        // Application de la formule Kp = (2*Pi - Somme_Angles) / Somme_Aires
        if (sum_areas > 1e-10) {
            Kcourb[v] = (2 * M_PI - sum_angles) / sum_areas;
        } else {
            Kcourb[v] = 0.0;
        }
    }
    std::cout << "Courubres gaussiennes calculees." << std::endl;
}

void color_mesh_by_curvature(Mesh &m) {
    // Calculer les courbes
    // On crée la propriété pour stocker les valeurs brutes
    Mesh::Property_map<Mesh::Vertex_index, double> K_prop;
    bool created;
    boost::tie(K_prop, created) = m.add_property_map<Mesh::Vertex_index, double>("v:curvature", 0.0);


    // On appelle la fonction de calcul des courbures
    gaussian_curv(m, K_prop);

    // Statistique avec la classe distrib
    distrib<double> d; // Instanciation

    for (auto v : m.vertices()) {
        // On exclut souvent les  bords car leur courbure est mal définie
        if (!m.is_border(v)) {
            d.add_data(K_prop[v]);
        }
    }

    // Récupération des stats
    double kmean = d.get_mean();
    double kmin = d.get_min();
    double kmax = d.get_max();
    double kstdev = d.get_stdev(); // Appel unique important vu le code de la classe

    std::cout << "Courbure Stats (via distrib.hpp) -> Moyenne : " << kmean << ", Min : " << kmin << ", Max : " << kmax << ", Ecart-type : " << kstdev << std::endl;

    // Création de la propriété couleur
    Mesh::Property_map<Mesh::Vertex_index, CGAL::Color> color_prop;
    boost::tie(color_prop, created) = m.add_property_map<Mesh::Vertex_index, CGAL::Color>("v:color");

    // Définition des couleurs extrème
    CGAL::Color col_bleu = CGAL::blue(); // Pour les valeurs faibles /négatives
    CGAL::Color col_rouge = CGAL::red(); // Pour les valeurs fortes /positives

    for (auto v : m.vertices()) {
        if (m.is_border(v)) {
            // On met les bords en noir ou vert pour les ignorer visuellement
            color_prop[v] = CGAL::Color(0, 0, 0);
        } else {
            // Utilisation de la fonction color_ramp
            // Elle va mapper la valeur k_prop[v] sur le gradient Bleu-blanc-Rouge
            // en se basant sur la moyenne et l'écart-type calculés par distrib
            color_prop[v] = color_ramp(kmin, kmax, kmean, kstdev, K_prop[v], col_bleu, col_rouge);
        }
    }

    std::cout << "Coloration terminee. " << std::endl;
}