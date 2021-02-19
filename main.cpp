#include "graph/Graph.hpp"
#include "graph/algorithm.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <utlRandom.hpp>
#include <utlTime.hpp>

// https://algs4.cs.princeton.edu/41graph/

struct Data {
    float x;
    float y;

    Data()
    {
    }
    
    Data(const float x, const float y)
        : x(x)
        , y(y)
    {
    }

    bool operator==(const Data& o)
    {
        return x == o.x && y == y;
    }

    bool operator<(const Data& o)
    {
        return x < o.x || y < y;
    }

    bool operator!=(const Data& o)
    {
        return x != o.x || y != y;
    }
};

void genGraph()
{
    std::string file = "test2";
    std::size_t N = 50;
    std::size_t RND = 3;
    std::ofstream outGz;
    std::ofstream outFile;
    std::size_t count = 0;
    std::map<std::pair<std::size_t, std::size_t>, std::size_t> map;
    outGz.open(file + ".gv");
    outFile.open(file + ".txt");
    outGz << "graph {\n";
    for(std::size_t i = 0; i < N; ++i) {
        std::size_t a = i;
        std::size_t b = utl::random::getUniformInt(0, RND);
        for(std::size_t j = 0; j < b; ++j) {
            std::size_t c = utl::random::getUniformInt(0, N);
            if(a != c) {
                if(map.find(std::make_pair(a, c)) == map.end() && map.find(std::make_pair(c, a)) == map.end()) {
                    outGz << "\t" << a << " -- " << c << ";\n";
                    outFile << a << " " << c << "\n";
                    map[std::make_pair(a, c)] = 1;
                    map[std::make_pair(c, a)] = 1;
                    count++;
                }
            }
        }
    }
    outGz << "}\n";
    outGz.close();
    outFile.close();
    std::cout << count << " pairs generated" << std::endl;
}

void toDot()
{
    std::ifstream inFile("mediumG.txt");
    std::ofstream outFile;
    outFile.open("medium.gv");
    std::size_t a, b;
    outFile << "graph {\n";
    while(inFile >> a >> b) {
        outFile << "\t" << a << " -- " << b << ";\n";
    }
    outFile << "}\n";
    inFile.close();
    outFile.close();
}

void testGraphString()
{
    grf::Graph<std::string> gStr(grf::GraphType::Undirected);
    gStr.addEdge("A", "B");
    gStr.addEdge("A", "C");
    gStr.addEdge("A", "E");
    gStr.addEdge("B", "D");
    gStr.addEdge("B", "F");
    gStr.addEdge("C", "G");
    gStr.addEdge("E", "F");

    std::cout << gStr << std::endl;
    std::cout << "Graph size: " << gStr.size() << " edges size: " << gStr.edgesCount() << std::endl;

    std::vector<std::string*> vertices = grf::dfs<std::string>(gStr, "A");
    std::cout << "DFS (shoud be A, B, D, F, C, G, E):" << std::endl;
    for(auto& v : vertices)
        std::cout << *v << " ";
    std::cout << std::endl << std::endl;

    vertices = grf::bfs<std::string>(gStr, "A");
    std::cout << "BFS (should be A, B, C, E, D, F, G) :" << std::endl;
    for(auto& v : vertices)
        std::cout << *v << " ";
    std::cout << std::endl << std::endl;

    std::cout << "size before removing vertex B " << gStr.size() << std::endl;
    gStr.removeVertex("B");
    std::cout << "size after removing vertex B " << gStr.size() << std::endl;
    std::cout << gStr << std::endl;
}

void testDijkstra()
{
    grf::Graph<std::size_t> g;
    g.addEdge(0, 1, 4.f);
    g.addEdge(0, 7, 8.f);
    g.addEdge(1, 2, 8.f);
    g.addEdge(1, 7, 11.f);
    g.addEdge(2, 3, 7.f);
    g.addEdge(2, 8, 2.f);
    g.addEdge(2, 5, 4.f);
    g.addEdge(3, 4, 9.f);
    g.addEdge(3, 5, 14.f);
    g.addEdge(4, 5, 10.f);
    g.addEdge(5, 6, 2.f);
    g.addEdge(6, 7, 1.f);
    g.addEdge(6, 8, 6.f);
    g.addEdge(7, 8, 7.f);

    std::vector<std::size_t*> res = grf::dijkstra<std::size_t>(g, 8, 4);
    for(auto& r : res)
        std::cout << *r << " ";
    std::cout << std::endl;
}

void testBfsPath()
{
    grf::Graph<std::size_t> g;
    std::cout << "Create vertices" << std::endl;
    auto start = utl::time::getTimePoint();
    {
        std::ifstream file("test2.txt");
        std::size_t a, b;
        while(file >> a >> b) {
            g.addEdge(a, b);
        }
        file.close();
    }
    std::cout << "time: " << utl::time::elapsedTime(start, utl::time::getTimePoint()) << " ns, graph size: " << g.size()
              << std::endl
              << std::endl;

    {
        auto start = utl::time::getTimePoint();
        std::vector<std::size_t*> vertices = grf::bfsPath<std::size_t>(g, 7, 38);
        std::cout << "bfs path time: " << utl::time::elapsedTime(start, utl::time::getTimePoint())
                  << " ns, size:" << vertices.size() << std::endl;
        for(auto& v : vertices)
            std::cout << *v << " ";
        std::cout << std::endl;
    }
}

void testBigGraph()
{
    grf::Graph<std::size_t> g;
    std::cout << "Create vertices" << std::endl;
    auto start = utl::time::getTimePoint();
    {
        std::ifstream file("largeG.txt");
        std::size_t a, b;
        while(file >> a >> b) {
            g.addEdge(a, b);
        }
        file.close();
    }
    std::cout << "time: " << utl::time::elapsedTime(start, utl::time::getTimePoint()) << " ns, graph size: " << g.size()
              << std::endl;
}

void testGraphOperations()
{
    grf::Graph<std::string> g(grf::GraphType::Undirected);
    g.addEdge("A", "B");
    g.addEdge("A", "C");
    g.addEdge("A", "E");
    g.addEdge("B", "D");
    g.addEdge("B", "F");
    g.addEdge("C", "G");
    g.addEdge("E", "F");

    std::vector<std::string*> vertices = grf::bfsPath<std::string>(g, "D", "G");
    for(auto& v : vertices)
        std::cout << *v << " ";
    std::cout << std::endl << std::endl;

    std::cout << g << std::endl;
    std::cout << "Size: " << g.size() << " edges: " << g.edgesCount() << std::endl << std::endl;

    g.removeVertex("C");
    std::cout << "Remove vertex C" << std::endl;
    std::cout << g << std::endl;
    std::cout << "Size: " << g.size() << " edges: " << g.edgesCount() << std::endl << std::endl;

    g.removeEdge("B", "F");
    std::cout << "Remove edge B-F" << std::endl;
    std::cout << g << std::endl;
    std::cout << "Size: " << g.size() << " edges: " << g.edgesCount() << std::endl << std::endl;

    g.addEdge("C", "G");
    g.addEdge("A", "C");
    std::cout << g << std::endl;
    std::cout << "Size: " << g.size() << " edges: " << g.edgesCount() << std::endl << std::endl;

    grf::bfs<std::string>(g, "A");
}

int main(int argc, char** argv)
{

    std::cout << "Graph operations" << std::endl;
    testGraphOperations();
    std::cout << std::endl;
    std::cout << "BFS" << std::endl;
    testBfsPath();
    std::cout << std::endl;
    std::cout << "Dijkstra" << std::endl;
    testDijkstra();

    return 0;
}
