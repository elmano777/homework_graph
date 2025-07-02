//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <set>


// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    Dijkstra,
    BFS,
    AStar
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    struct Entry {
        Node* node;
        double dist;

        bool operator < (const Entry& other) const {
            return dist < other.dist;
        }
    };

    double heuristic(Node* a, Node* b) {
        float dx = a->coord.x - b->coord.x;
        float dy = a->coord.y - b->coord.y;
        return std::sqrt(dx * dx + dy * dy);
    }


    void dijkstra(Graph &graph) {
        std::unordered_map<Node*, double> dist;
        std::unordered_map<Node*, Node*> parent;
        std::set<std::pair<double, Node*>> pq;

        // Inicializar distancias
        for (auto &[_, node] : graph.nodes) {
            dist[node] = std::numeric_limits<double>::infinity();
        }

        dist[src] = 0.0;
        pq.insert({0.0, src});

        while (!pq.empty()) {
            auto [curr_dist, current] = *pq.begin();
            pq.erase(pq.begin());

            if (current == dest) break;

            for (Edge *edge : current->edges) {
                Node* neighbor = (edge->src == current) ? edge->dest : edge->src;
                double weight = edge->length;

                if (dist[current] + weight < dist[neighbor]) {
                    pq.erase({dist[neighbor], neighbor}); // actualiza si ya estaba
                    dist[neighbor] = dist[current] + weight;
                    parent[neighbor] = current;
                    pq.insert({dist[neighbor], neighbor});

                    visited_edges.emplace_back(current->coord, neighbor->coord, sf::Color(100, 100, 255), 0.6f);
                    render();
                }
            }
        }

        set_final_path(parent);
    }


    void bfs(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        // TODO: Add your code here

        set_final_path(parent);
    }

    void a_star(Graph &graph) {
        std::unordered_map<Node*, double> g_score;
        std::unordered_map<Node*, double> f_score;
        std::unordered_map<Node*, Node*> parent;
        std::set<std::pair<double, Node*>> open_set;

        for (auto &[_, node] : graph.nodes) {
            g_score[node] = std::numeric_limits<double>::infinity();
            f_score[node] = std::numeric_limits<double>::infinity();
        }

        g_score[src] = 0.0;
        f_score[src] = heuristic(src, dest);
        open_set.insert({f_score[src], src});

        while (!open_set.empty()) {
            auto [curr_f, current] = *open_set.begin();
            open_set.erase(open_set.begin());

            if (current == dest) break;

            for (Edge *edge : current->edges) {
                Node *neighbor = (edge->src == current) ? edge->dest : edge->src;
                double tentative_g = g_score[current] + edge->length;

                if (tentative_g < g_score[neighbor]) {
                    open_set.erase({f_score[neighbor], neighbor}); // actualiza si ya estaba

                    parent[neighbor] = current;
                    g_score[neighbor] = tentative_g;
                    f_score[neighbor] = tentative_g + heuristic(neighbor, dest);
                    open_set.insert({f_score[neighbor], neighbor});

                    visited_edges.emplace_back(current->coord, neighbor->coord, sf::Color(255, 100, 100), 0.6f);
                    render();
                }
            }
        }

        set_final_path(parent);
    }


    void render() {
        sf::sleep(sf::milliseconds(10)); // pequeña pausa para visualizar

        window_manager->clear();          // limpia la ventana
        window_manager->get_window().setFramerateLimit(60);

        // Dibuja todo el grafo
        // Importante: asegurarse de que todavía tienes acceso al grafo en esta función
        // Como no lo tienes directamente, lo ideal sería pasar el grafo como parámetro. Pero si no quieres cambiar la firma, esto es opcional.

        // Dibuja aristas visitadas (hasta ahora)
        for (sfLine &line : visited_edges) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibuja el nodo fuente y destino si existen
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }

        // Muestra el frame
        window_manager->display();
    }


    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node *, Node *> &parent) {
        path.clear();
        Node* current = dest;

        while (parent.find(current) != parent.end()) {
            Node* prev = parent[current];
            path.emplace_back(current->coord, prev->coord, sf::Color::White, 2.0f);
            current = prev;
        }
    }


public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) return;

        switch (algorithm) {
            case Dijkstra:
                dijkstra(graph);
                break;
            case BFS:
                bfs(graph);
                break;
            case AStar:
                a_star(graph);
                break;
            default:
                break;
        }
    }



    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
