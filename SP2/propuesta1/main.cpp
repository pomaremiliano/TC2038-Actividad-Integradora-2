/*
<------------------------------------------------------------------------------------->
<-------------------Propuesta 1 de la Actividad Integradora 2------------------------->
<------------------------------------------------------------------------------------->

Integrantes:
- Luis Gabriel Delfín Paulín | A01701482
- Jorge Emiliano Pomar Mendoza | A01709338
- Eliuth Balderas Neri | A01703315 

<------------------------------------------------------------------------------------->
<-----------------------Para compilar y correr con los casos de prueba---------------->
<------------------------------------------------------------------------------------->

  Para compilar el programa, se debe de correr el siguiente comando en la terminal:

    g++ -std=c++17 -o main main.cpp

  Para correr el programa, se debe de correr el siguiente comando en la terminal:

    ./main < entradas/entrada1.txt && ./main < entradas/entrada2.txt && ./main < entradas/entrada3.txt

*/

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <iostream>
#include <queue>
#include <sstream>
#include <utility>
#include <vector>

// Estructura edge para representar las aristas del grafo
struct Edge {
  int src, dest, weight;
};

// Estructura Graph para representar el grafo con V vértices y E aristas
struct Graph {
  int V, E;
  std::vector<Edge> edges;

  Graph(int V, int E) : V(V), E(E) {}

  void addEdge(int src, int dest, int weight) {
    edges.push_back({src, dest, weight});
  }
};

// Estructura UnionFind para implementar la estructura de datos de conjuntos
// disjuntos
struct UnionFind {
  std::vector<int> parent, rank;
  UnionFind(int size) : parent(size), rank(size, 0) {
    for (int i = 0; i < size; i++)
      parent[i] = i;
  }
  int find(int i) {
    if (parent[i] != i)
      parent[i] = find(parent[i]);
    return parent[i];
  }

  void unionSet(int x, int y) {
    int rootX = find(x);
    int rootY = find(y);

    if (rootX != rootY) {
      if (rank[rootX] > rank[rootY]) {
        parent[rootY] = rootX;
      } else if (rank[rootX] < rank[rootY]) {
        parent[rootX] = rootY;
      } else {
        parent[rootY] = rootX;
        rank[rootX]++;
      }
    }
  }
};

// Para Kruskal primero se ordenan las aristas por peso
bool compEdge(const Edge &e1, const Edge &e2) { return e1.weight < e2.weight; }

// Función para imprimir el grafo min spanning tree
void printGraph(std::vector<Edge> &spanningTree) {
  for (auto &e : spanningTree) {
    std::cout << char('A' + e.src) << " - " << char('A' + e.dest) << " ("
              << e.weight << " km)" << std::endl;
  }
}

// Kruskal para encontrar el MST arbol de expansion minima del grafo
std::vector<Edge> kruskalMST(Graph &graph) {
  std::vector<Edge> mst;
  sort(graph.edges.begin(), graph.edges.end(), compEdge);
  UnionFind uf(graph.V);

  for (auto &edge : graph.edges) {
    if (uf.find(edge.src) != uf.find(edge.dest)) {
      uf.unionSet(edge.src, edge.dest);
      mst.push_back(edge);
    }
  }
  return mst;
}

// TSP con el algoritmo del vecino más cercano (Nearest Neighbor) para la ruta
// más corta que visita cada colonia exactamente 1 vez
std::vector<int> tspNearestNeighbor(const Graph &graph, int start) {
  int n = graph.V;
  std::vector<bool> visited(n, false);
  std::vector<int> path;
  int current = start;
  path.push_back(current);
  visited[current] = true;
  int total_distance = 0;

  for (int i = 0; i < n - 1; ++i) {
    int nearest = -1;
    int nearest_dist = std::numeric_limits<int>::max();
    for (int j = 0; j < n; ++j) {
      if (!visited[j] && graph.edges[current * n + j].weight > 0 &&
          graph.edges[current * n + j].weight < nearest_dist) {
        nearest_dist = graph.edges[current * n + j].weight;
        nearest = j;
      }
    }
    if (nearest != -1) {
      path.push_back(nearest);
      visited[nearest] = true;
      total_distance += nearest_dist;
      current = nearest;
    }
  }
  total_distance += graph.edges[current * n + start].weight;
  path.push_back(start);
  std::cout << "Ruta del Viajero:" << std::endl;
  for (int i : path) {
    std::cout << char('A' + i) << " ";
  }
  std::cout << "\nDistancia total: " << total_distance << " km" << std::endl;

  return path;
}

// Primero se hace un BFS para encontrar un camino que va en aumento y con
// capacidad residual mayor a 0
std::vector<int> bfsEdmondsKarp(Graph &graph, int src, int dest,
                                std::vector<std::vector<int>> &residualGraph) {
  std::vector<int> parent(graph.V, -1);
  std::vector<bool> visited(graph.V, false);

  std::queue<int> path;
  path.push(src);
  visited[src] = true;

  while (!path.empty()) {
    int u = path.front();
    path.pop();

    for (int v = 0; v < graph.V; v++) {
      if (!visited[v] && residualGraph[u][v] > 0) {
        parent[v] = u;
        visited[v] = true;
        path.push(v);
        if (v == dest)
          return parent;
      }
    }
  }

  return parent;
}

// Con el grafo de arriba se calcula el flujo máximo entre la colonia 0 y la
// colonia N-1
int edmondsKarp(Graph &graph, int src, int dest) {
  std::vector<std::vector<int>> residualGraph(graph.V,
                                              std::vector<int>(graph.V, 0));
  for (auto &edge : graph.edges) {
    residualGraph[edge.src][edge.dest] = edge.weight;
  }

  int maxFlow = 0;
  while (true) {
    std::vector<int> parent = bfsEdmondsKarp(graph, src, dest, residualGraph);
    if (parent[dest] == -1) {
      break;
    }

    int pathFlow = INT_MAX;
    for (int v = dest; v != src; v = parent[v]) {
      int u = parent[v];
      pathFlow = std::min(pathFlow, residualGraph[u][v]);
    }

    for (int v = dest; v != src; v = parent[v]) {
      int u = parent[v];
      residualGraph[u][v] -= pathFlow;
      residualGraph[v][u] += pathFlow;
    }

    maxFlow += pathFlow;
  }

  return maxFlow;
}

// Calcula la distancia minima entre la central y las colonias usando la
// distancia euclidiana
int distanciaMinimaEuclidiana(std::pair<int, int> src,
                              std::pair<int, int> dest) {
  return std::sqrt(std::pow(src.first - dest.first, 2) +
                   std::pow(src.second - dest.second, 2));
}

// Para comparar, vamos a calcular la distancia minima entre la central y las
// colonias usando la distancia de Manhattan
int distanciaMinimaManhattan(int src, int dest,
                             std::vector<std::pair<int, int>> &coordinates) {
  return std::abs(coordinates[src].first - coordinates[dest].first) +
         std::abs(coordinates[src].second - coordinates[dest].second);
}

int main() {
  // Leer el número de colonias
  int N;
  std::cin >> N;

  // Crear un grafo para las distancias y para los flujos
  Graph distanceGraph(N, N * (N - 1) / 2);
  Graph flowGraph(N, N * (N - 1) / 2);

  // Leer la matriz de distancias
  int dist;
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      std::cin >> dist;
      if (i != j) {
        distanceGraph.addEdge(i, j, dist);
      }
    }
  }

  // Leer la matriz de flujos
  int flow;
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      std::cin >> flow;
      if (i != j) {
        flowGraph.addEdge(i, j, flow);
      }
    }
  }

  // Leer las coordenadas de las centrales existentes
  std::vector<std::pair<int, int>> coordinates;
  std::string line;
  std::getline(std::cin,
               line); // Leer la línea vacía que precede a las coordenadas
  for (int i = 0; i < N; ++i) {
    std::getline(std::cin, line);
    std::istringstream iss(line);
    int x, y;
    char delim; // Para leer las comas
    iss >> delim >> x >> delim >> y >> delim;
    coordinates.push_back({x, y});
  }

  // Leer la coordenada de la nueva central
  std::pair<int, int> new_central;
  std::getline(std::cin, line);
  std::istringstream iss(line);
  char delim;
  iss >> delim >> new_central.first >> delim >> new_central.second >> delim;

  // Calcular el MST utilizando Kruskal
  auto start = std::chrono::high_resolution_clock::now();
  std::vector<Edge> mst = kruskalMST(distanceGraph);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "=====================" << std::endl;
  std::cout << "====== Parte 1 ======" << std::endl;
  std::cout << "=====================" << std::endl;
  std::cout << "Forma óptima de cablear las colonias:" << std::endl;
  printGraph(mst);
  std::cout << "Tiempo de ejecución (Kruskal): " << elapsed.count() * 1000
            << " ms" << std::endl;

  // Calcular el reparto de correspondencia entre colonias
  std::cout << "=====================" << std::endl;
  std::cout << "====== Parte 2 ======" << std::endl;
  std::cout << "=====================" << std::endl;
  start = std::chrono::high_resolution_clock::now();
  std::vector<int> correspondencia = tspNearestNeighbor(flowGraph, 0);
  end = std::chrono::high_resolution_clock::now();
  elapsed = end - start;
  std::cout << "Tiempo de ejecución (TSP - Nearest Neighbor): "
            << elapsed.count() * 1000 << " ms" << std::endl;

  // Calcular el flujo máximo entre las colonias 0 y N-1
  std::cout << "=====================" << std::endl;
  std::cout << "====== Parte 3 ======" << std::endl;
  std::cout << "=====================" << std::endl;
  start = std::chrono::high_resolution_clock::now();
  int maxFlow = edmondsKarp(flowGraph, 0, N - 1);
  end = std::chrono::high_resolution_clock::now();
  elapsed = end - start;
  std::cout << "Flujo máximo entre las colonias 0 y " << N - 1 << ": "
            << maxFlow << " personas" << std::endl;
  std::cout << "Tiempo de ejecución (Edmonds-Karp): " << elapsed.count() * 1000
            << " ms" << std::endl;

  // Calcular la distancia mínima entre la nueva central y las colonias
  // existentes
  std::cout << "=====================" << std::endl;
  std::cout << "====== Parte 4 ======" << std::endl;
  std::cout << "=====================" << std::endl;
  start = std::chrono::high_resolution_clock::now();
  std::vector<int> distancias;
  for (int i = 0; i < N; ++i) {
    distancias.push_back(
        distanciaMinimaEuclidiana(coordinates[i], new_central));
  }
  end = std::chrono::high_resolution_clock::now();
  elapsed = end - start;
  std::cout << "Distancia mínima entre la nueva central y las colonias "
               "existentes (euclidiana):"
            << std::endl;
  for (int i = 0; i < N; ++i) {
    std::cout << char('A' + i) << ": " << distancias[i] << " km" << std::endl;
  }
  std::cout << "Tiempo de ejecución (Distancia Euclidiana): "
            << elapsed.count() * 1000 << " ms" << std::endl;

  // Con la distancia de Manhattan
  start = std::chrono::high_resolution_clock::now();
  std::vector<int> distanciasManhattan;
  for (int i = 0; i < N; ++i) {
    distanciasManhattan.push_back(distanciaMinimaManhattan(i, N, coordinates));
  }
  end = std::chrono::high_resolution_clock::now();
  elapsed = end - start;
  std::cout << "Distancia mínima entre la nueva central y las colonias "
               "existentes (Manhattan):"
            << std::endl;
  for (int i = 0; i < N; ++i) {
    std::cout << char('A' + i) << ": " << distanciasManhattan[i] << " km"
              << std::endl;
  }
  std::cout << "Tiempo de ejecución (Distancia Manhattan): "
            << elapsed.count() * 1000 << " ms" << std::endl;
  return 0;
}
