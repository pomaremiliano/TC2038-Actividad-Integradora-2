/*
<------------------------------------------------------------------------------------->
<-------------------Propuesta 2 de la Actividad Integradora 2------------------------->
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

// Estructura Edge para representar las aristas del grafo
struct Edge {
  int src, dest, weight;
};

// Estructura Graph para representar el grafo con V vértices y matriz de adyacencia
struct Graph {
  int V, E;
  std::vector<std::vector<int>> adjMatrix;
  Graph(int V) : V(V), E(0) {
    adjMatrix.resize(V, std::vector<int>(V, 0));
  }

  // Función para agregar una arista al grafo
  void addEdge(int src, int dest, int weight) {
    adjMatrix[src][dest] = weight;
    adjMatrix[dest][src] = weight;
    E++;
  }
};

// Estructura MinHeapNode para representar un nodo del min heap
struct MinHeapNode {
  int v;
  int key;
};

// Estructura MinHeap para implementar un min heap
struct MinHeap {
  int size;
  int capacity;
  std::vector<int> pos;
  std::vector<MinHeapNode> array;
  MinHeap(int capacity) : size(0), capacity(capacity), pos(capacity), array(capacity) {}

  void swapMinHeapNode(MinHeapNode* a, MinHeapNode* b) {
    MinHeapNode t = *a;
    *a = *b;
    *b = t;
  }

  void minHeapify(int idx) {
    int smallest, left, right;
    smallest = idx;
    left = 2 * idx + 1;
    right = 2 * idx + 2;

    if (left < size && array[left].key < array[smallest].key)
      smallest = left;

    if (right < size && array[right].key < array[smallest].key)
      smallest = right;

    if (smallest != idx) {
      MinHeapNode smallestNode = array[smallest];
      MinHeapNode idxNode = array[idx];

      pos[smallestNode.v] = idx;
      pos[idxNode.v] = smallest;

      swapMinHeapNode(&array[smallest], &array[idx]);
      minHeapify(smallest);
    }
  }

  MinHeapNode extractMin() {
    if (size == 0)
      return {0, 0};

    MinHeapNode root = array[0];
    MinHeapNode lastNode = array[size - 1];
    array[0] = lastNode;

    pos[root.v] = size - 1;
    pos[lastNode.v] = 0;

    --size;
    minHeapify(0);

    return root;
  }

  void decreaseKey(int v, int key) {
    int i = pos[v];
    array[i].key = key;

    while (i && array[i].key < array[(i - 1) / 2].key) {
      pos[array[i].v] = (i - 1) / 2;
      pos[array[(i - 1) / 2].v] = i;
      swapMinHeapNode(&array[i], &array[(i - 1) / 2]);
      i = (i - 1) / 2;
    }
  }

  bool isInMinHeap(int v) {
    return pos[v] < size;
  }
};

// Función para imprimir el grafo del árbol de expansión mínima
void printArr(std::vector<int>& parent, std::vector<std::vector<int>>& graph, int V) {
  for (int i = 1; i < V; ++i)
    std::cout << char('A' + parent[i]) << " - " << char('A' + i) << " (" << graph[i][parent[i]] << " km)" << std::endl;
}

// Algoritmo de Prim para encontrar el MST (árbol de expansión mínima) del grafo
// basado en https://www.geeksforgeeks.org/prims-minimum-spanning-tree-mst-greedy-algo-5/
void primMST(Graph& graph) {
  int V = graph.V;
  std::vector<int> parent(V); 
  std::vector<int> key(V, INT_MAX); 
  MinHeap minHeap(V); 

  for (int v = 0; v < V; ++v) {
    minHeap.array[v] = {v, key[v]};
    minHeap.pos[v] = v;
  }

  minHeap.pos[0] = 0;
  key[0] = 0;
  minHeap.array[0] = {0, key[0]};
  minHeap.size = V;

  parent[0] = -1; 

  while (minHeap.size > 0) {
    MinHeapNode minHeapNode = minHeap.extractMin();
    int u = minHeapNode.v;

    for (int v = 0; v < V; ++v) {
      if (graph.adjMatrix[u][v] && minHeap.isInMinHeap(v) && graph.adjMatrix[u][v] < key[v]) {
        key[v] = graph.adjMatrix[u][v];
        parent[v] = u;
        minHeap.decreaseKey(v, key[v]);
      }
    }
  }
  printArr(parent, graph.adjMatrix, V); 
}

// Estructura TSPNode para representar un nodo en el algoritmo de Ramificación y Acotamiento
struct TSPNode {
  std::vector<int> path;
  int cost;
  int bound;
};

// Estructura CompareTSPNode para comparar nodos en el algoritmo de Ramificación y Acotamiento
struct CompareTSPNode {
  bool operator()(TSPNode const& n1, TSPNode const& n2) {
    return n1.bound > n2.bound;
  }
};

// Función para calcular la cota inferior de un nodo en el algoritmo de Ramificación y Acotamiento
int calculateBound(const TSPNode& node, const Graph& graph, int N) {
  int bound = node.cost;
  for (int i = 0; i < N; ++i) {
    if (std::find(node.path.begin(), node.path.end(), i) == node.path.end()) {
      int minCost = INT_MAX;
      for (int j = 0; j < N; ++j) {
        if (i != j && std::find(node.path.begin(), node.path.end(), j) == node.path.end()) {
          minCost = std::min(minCost, graph.adjMatrix[i][j]);
        }
      }
      bound += minCost;
    }
  }
  return bound;
}

// Algoritmo de Ramificación y Acotamiento para resolver el problema del viajero
void branchAndBoundTSP(const Graph& graph, int start) {
  int N = graph.V;
  std::priority_queue<TSPNode, std::vector<TSPNode>, CompareTSPNode> pq;
  TSPNode root = {{start}, 0, 0};
  root.bound = calculateBound(root, graph, N);
  pq.push(root);

  TSPNode bestNode;
  bestNode.cost = INT_MAX;

  while (!pq.empty()) {
    TSPNode current = pq.top();
    pq.pop();

    if (current.bound < bestNode.cost) {
      for (int i = 0; i < N; ++i) {
        if (std::find(current.path.begin(), current.path.end(), i) == current.path.end()) {
          TSPNode child = current;
          child.path.push_back(i);
          child.cost += graph.adjMatrix[current.path.back()][i];
          if (child.path.size() == N) {
            child.path.push_back(start);
            child.cost += graph.adjMatrix[i][start];
            if (child.cost < bestNode.cost) {
              bestNode = child;
            }
          } else {
            child.bound = calculateBound(child, graph, N);
            if (child.bound < bestNode.cost) {
              pq.push(child);
            }
          }
        }
      }
    }
  }

  std::cout << "Ruta del Viajero:" << std::endl;
  for (int i : bestNode.path) {
    std::cout << char('A' + i) << " ";
  }
  std::cout << "\nDistancia total: " << bestNode.cost << " km" << std::endl;
}

// Estructura PushRelabel para implementar el algoritmo de flujo máximo Push-Relabel
// basado en https://www.geeksforgeeks.org/push-relabel-algorithm-set-2-implementation/
struct PushRelabel {
  int V;
  std::vector<std::vector<int>> capacity;
  std::vector<std::vector<int>> flow;
  std::vector<int> height;
  std::vector<int> excess;

  PushRelabel(int V) : V(V), capacity(V, std::vector<int>(V, 0)), flow(V, std::vector<int>(V, 0)), height(V, 0), excess(V, 0) {}

  void addEdge(int u, int v, int cap) {
    capacity[u][v] += cap;
  }

  void push(int u, int v) {
    int send = std::min(excess[u], capacity[u][v] - flow[u][v]);
    flow[u][v] += send;
    flow[v][u] -= send;
    excess[u] -= send;
    excess[v] += send;
  }

  void relabel(int u) {
    int minHeight = INT_MAX;
    for (int v = 0; v < V; ++v) {
      if (capacity[u][v] - flow[u][v] > 0) {
        minHeight = std::min(minHeight, height[v]);
        height[u] = minHeight + 1;
      }
    }
  }

  void discharge(int u) {
    while (excess[u] > 0) {
      bool pushed = false;
      for (int v = 0; v < V; ++v) {
        if (capacity[u][v] - flow[u][v] > 0 && height[u] == height[v] + 1) {
          push(u, v);
          pushed = true;
        }
      }
      if (!pushed) {
        relabel(u);
      }
    }
  }

  int getMaxFlow(int s, int t) {
    height[s] = V;
    excess[s] = 0;
    for (int v = 0; v < V; ++v) {
      if (capacity[s][v] > 0) {
        flow[s][v] = capacity[s][v];
        flow[v][s] = -flow[s][v];
        excess[v] += flow[s][v];
        excess[s] -= flow[s][v];
      }
    }

    std::queue<int> active;
    for (int i = 0; i < V; ++i) {
      if (i != s && i != t && excess[i] > 0) {
        active.push(i);
      }
    }

    while (!active.empty()) {
      int u = active.front();
      active.pop();
      discharge(u);
      if (excess[u] > 0) {
        active.push(u);
      }
    }

    return excess[t];
  }
};

// Calcula la distancia mínima entre la central y las colonias usando la distancia euclidiana
int distanciaMinimaEuclidiana(std::pair<int, int> src, std::pair<int, int> dest) {
  return std::sqrt(std::pow(src.first - dest.first, 2) + std::pow(src.second - dest.second, 2));
}

// Calcula la distancia mínima entre la central y las colonias usando la distancia de Manhattan
int distanciaMinimaManhattan(int src, int dest, std::vector<std::pair<int, int>>& coordinates) {
  return std::abs(coordinates[src].first - coordinates[dest].first) + std::abs(coordinates[src].second - coordinates[dest].second);
}

int main() {
  int N;
  std::cin >> N;

  // Crear un grafo para las distancias y para los flujos
  Graph distanceGraph(N);
  Graph flowGraph(N);

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
  PushRelabel pushRelabel(N);
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      std::cin >> flow;
      if (i != j) {
        pushRelabel.addEdge(i, j, flow);
      }
    }
  }

  // Leer las coordenadas de las centrales existentes
  std::vector<std::pair<int, int>> coordinates;
  std::string line;
  std::getline(std::cin, line); // Leer la línea vacía que precede a las coordenadas
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

  // Calcular el MST utilizando Prim
  auto start = std::chrono::high_resolution_clock::now();
  std::cout << "=====================" << std::endl;
  std::cout << "====== Parte 1 ======" << std::endl;
  std::cout << "=====================" << std::endl;
  std::cout << "Forma óptima de cablear las colonias:" << std::endl;
  primMST(distanceGraph);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "Tiempo de ejecución (Prim): " << elapsed.count() * 1000 << " ms" << std::endl;

  // Calcular el reparto de correspondencia entre colonias usando Branch and Bound
  std::cout << "=====================" << std::endl;
  std::cout << "====== Parte 2 ======" << std::endl;
  std::cout << "=====================" << std::endl;
  start = std::chrono::high_resolution_clock::now();
  branchAndBoundTSP(distanceGraph, 0);
  end = std::chrono::high_resolution_clock::now();
  elapsed = end - start;
  std::cout << "Tiempo de ejecución (TSP - Branch and Bound): " << elapsed.count() * 1000 << " ms" << std::endl;

  // Calcular el flujo máximo entre las colonias 0 y N-1 usando Push-Relabel
  std::cout << "=====================" << std::endl;
  std::cout << "====== Parte 3 ======" << std::endl;
  std::cout << "=====================" << std::endl;
  start = std::chrono::high_resolution_clock::now();
  int maxFlow = pushRelabel.getMaxFlow(0, N - 1);
  end = std::chrono::high_resolution_clock::now();
  elapsed = end - start;
  std::cout << "Flujo máximo entre las colonias 0 y " << N - 1 << ": " << maxFlow << " personas" << std::endl;
  std::cout << "Tiempo de ejecución (Push-Relabel): " << elapsed.count() * 1000 << " ms" << std::endl;

  // Calcular la distancia mínima entre la nueva central y las colonias existentes
  std::cout << "=====================" << std::endl;
  std::cout << "====== Parte 4 ======" << std::endl;
  std::cout << "=====================" << std::endl;
  start = std::chrono::high_resolution_clock::now();
  std::vector<int> distancias;
  for (int i = 0; i < N; ++i) {
    distancias.push_back(distanciaMinimaEuclidiana(coordinates[i], new_central));
  }
  end = std::chrono::high_resolution_clock::now();
  elapsed = end - start;
  std::cout << "Distancia mínima entre la nueva central y las colonias existentes (euclidiana):" << std::endl;
  for (int i = 0; i < N; ++i) {
    std::cout << char('A' + i) << ": " << distancias[i] << " km" << std::endl;
  }
  std::cout << "Tiempo de ejecución (Distancia Euclidiana): " << elapsed.count() * 1000 << " ms" << std::endl;

  start = std::chrono::high_resolution_clock::now();
  std::vector<int> distanciasManhattan;
  for (int i = 0; i < N; ++i) {
    distanciasManhattan.push_back(distanciaMinimaManhattan(i, N, coordinates));
  }
  end = std::chrono::high_resolution_clock::now();
  elapsed = end - start;
  std::cout << "Distancia mínima entre la nueva central y las colonias existentes (Manhattan):" << std::endl;
  for (int i = 0; i < N; ++i) {
    std::cout << char('A' + i) << ": " << distanciasManhattan[i] << " km" << std::endl;
  }
  std::cout << "Tiempo de ejecución (Distancia Manhattan): " << elapsed.count() * 1000 << " ms" << std::endl;
  return 0;
}
