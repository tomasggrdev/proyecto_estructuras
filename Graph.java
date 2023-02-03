package utils;
import java.util.*;

public class Graph {

    //cantidad de vertices que tendrá el grafo creado
    private final int vertices;

    //matriz de enteros que representará los pesos de las aristas
    private static int[][] adj;

    //cantidad de aristas creadas
    private int edges;

    //comentario prueba
    /**
     * Constructor de la clase Graph
     * @param vertices representa la cantidad de vertices que tendrá el nuevo grafo.
     */
    public Graph(int vertices) {
        this.vertices = vertices;
        adj = new int[vertices][vertices];
        edges = 0;
    }


    /**
     * addEdge permite conectar dos vertices del nodo
     * @param v1 vértice de inicio
     * @param v2 vértice de destino
     * @param weight peso de la arista que conecta los vertices
     */
    public void addEdge(int v1, int v2, int weight){
        adj[v1][v2] = weight;
        adj[v2][v1] = weight;
        edges++;
    }


    //Imprime una matriz de pesos asociada al grafo creado
    public void print() {
        for (int i = 0; i < vertices; i++) {
            for (int j = 0; j < vertices; j++) {
                if(i == j){
                    System.out.print("0\t");
                }else{
                    if(adj[i][j] != 0){
                        System.out.print(adj[i][j] + "\t");
                    }else{
                        System.out.print("∞\t");
                    }

                }

            }
            System.out.println();
        }
    }


    /**
     *Obtener la cantidad de vertices con los que se creó el grafo
     * @return numero de vertices del grafo
     */
    public int getVertices() {
        return vertices;
    }


    /**
     * Obtener la matriz adjunta al grafo
     * @return matriz de adyacencia
     */
    public int[][] getAdj(){
        return adj;
    }

    /**
     * Recorrido DFS del grafo
     * @param v vértice de inicio del recorrido
     */
    public void DFS(int v) {
        Dfs(v, new boolean[vertices]);
    }


    /**
     * Recorrido DFS del grafo
     * @param v el vértice de inicio
     * @param visited lleva control de los vertices visitados
     */
    private void Dfs(int v, boolean[] visited){
        visited[v] = true;
        System.out.print(v + " ");
        for (int i = 0; i < vertices; i++) {
            if (adj[v][i] != 0 && !visited[i]) {
                Dfs(i, visited);
            }
        }
    }


    /**
     * recorrido BFS del grafo
     * @param v vértice de inicio
     */
    public void BFS(int v) {
        boolean[] visited = new boolean[vertices];
        Queue<Integer> queue = new LinkedList<>();
        visited[v] = true;
        queue.add(v);
        while (!queue.isEmpty()) {
            int current = queue.remove();
            System.out.print(current + " ");
            for (int i = 0; i < vertices; i++) {
                if (adj[current][i] != 0 && !visited[i]) {
                    visited[i] = true;
                    queue.add(i);
                }
            }
        }
        System.out.println();
    }

    //obtener cantidad total de aristas creadas
    public int getEdges() {
        return edges;
    }

    public List<Integer> TSP_DFS(int v, boolean[] visited, List<Integer> path, int target){
        path.add(v);
        visited[v] = true;
        if (v == target) {
            return path;
        }
        for (int i = 0; i < vertices; i++) {
            if (adj[v][i] != 0 && !visited[i]) {
                List<Integer> newPath = TSP_DFS(i, visited, path, target);
                if (newPath != null) {
                    return newPath;
                }
            }
        }
        visited[v] = false;
        path.remove(path.size() - 1);
        return null;
    }

    public List<Integer> TSP(int start, int target){
        List<Integer> path = new ArrayList<>();
        boolean[] visited = new boolean[vertices];
        return TSP_DFS(start, visited, path, target);
    }
}
