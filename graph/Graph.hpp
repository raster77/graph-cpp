#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace grf
{

enum GraphType { Directed, Undirected };

template <typename T> class Graph
{
    typedef typename std::unordered_map<T, std::vector<std::pair<T*, float>>>::iterator graphIterator;

public:
    Graph(GraphType graphType = GraphType::Undirected)
        : mGraphType(graphType)
        , mEdgesCount(0)
    {
    }

    ~Graph()
    {
    }

    void addVertex(const T& vertex)
    {
        adjacencyMap.try_emplace(vertex, std::vector<std::pair<T*, float>>{});
    }

    void addEdge(const T& vertexA, const T& vertexB, const float weight = 0.f)
    {
        auto [itVertexA, resA] = adjacencyMap.try_emplace(vertexA, std::vector<std::pair<T*, float>>{});
        if(!resA)
            itVertexA = adjacencyMap.find(vertexA);

        auto [itvertexB, resB] = adjacencyMap.try_emplace(vertexB, std::vector<std::pair<T*, float>>{});
        if(!resB)
            itvertexB = adjacencyMap.find(vertexB);

        auto itFind = std::find_if(itVertexA->second.begin(), itVertexA->second.end(),
            [itvertexB](const std::pair<T*, float>& edge) { return itvertexB->first == *edge.first; });

        if(itFind == itVertexA->second.end()) {
            itVertexA->second.emplace_back(std::make_pair(const_cast<T*>(&itvertexB->first), weight));
            mEdgesCount++;
            if(mGraphType == GraphType::Undirected) {
                itvertexB->second.emplace_back(std::make_pair(const_cast<T*>(&itVertexA->first), weight));
                mEdgesCount++;
            }
        }
    }

    void removeVertex(const T& vertex)
    {
        graphIterator it = adjacencyMap.find(vertex);
        if(it != adjacencyMap.end()) {
            for(auto& v : it->second) {
                graphIterator it2 = adjacencyMap.find(*v.first);
                auto itFind = std::find_if(it2->second.begin(), it2->second.end(),
                    [vertex](const std::pair<T*, float>& edge) { return vertex == *edge.first; });
                if(itFind != it2->second.end()) {
                    it2->second.erase(itFind);
                    mEdgesCount--;
                }
            }

            if(mGraphType == GraphType::Directed) {
                for(auto& k : adjacencyMap) {
                    auto itFind = std::find_if(k.second.begin(), k.second.end(),
                        [vertex](const std::pair<T*, float>& edge) { return vertex == *edge.first; });
                    if(itFind != k.second.end()) {
                        k.second.erase(itFind);
                        mEdgesCount--;
                    }
                }
            }
            mEdgesCount -= it->second.size();
            adjacencyMap.erase(it);
        }
    }

    void removeEdge(const T& vertexA, const T& vertexB)
    {
        graphIterator itA = adjacencyMap.find(vertexA);
        auto itB = adjacencyMap.find(vertexB);
        if(itA != adjacencyMap.end() && itB != adjacencyMap.end()) {
            auto it = std::find_if(itA->second.begin(), itA->second.end(),
                [vertexB](const std::pair<T*, float>& edge) { return vertexB == *edge.first; });
            if(it != itA->second.end()) {
                itA->second.erase(it);
                mEdgesCount--;
            }

            if(mGraphType == GraphType::Undirected) {
                it = std::find_if(itB->second.begin(), itB->second.end(),
                    [vertexA](const std::pair<T*, float>& edge) { return vertexA == *edge.first; });
                if(it != itB->second.end()) {
                    itB->second.erase(it);
                    mEdgesCount--;
                }
            }
        }
    }

    bool exists(const T& vertex)
    {
        return adjacencyMap.find(vertex) != adjacencyMap.end();
    }

    T* getVertex(const T& vertex)
    {
        T* res = nullptr;
        res = const_cast<T*>(&adjacencyMap.find(vertex)->first);
        return res;
    }

    const std::vector<std::pair<T*, float>>& getEdgesWithWeight(const T& vertex)
    {
        return adjacencyMap.find(vertex)->second;
    }

    std::vector<T*> getEdges(const T& vertex)
    {
        std::vector<T*> res;
        res.reserve(adjacencyMap.find(vertex)->second.size());
        std::transform(adjacencyMap.find(vertex)->second.begin(), adjacencyMap.find(vertex)->second.end(),
            std::back_inserter(res), [](const std::pair<T*, float>& p) { return p.first; });
        return res;
    }

    const std::size_t size() const
    {
        return adjacencyMap.size();
    }

    const std::size_t edgesCount() const
    {
        return mEdgesCount;
    }

    const GraphType& graphType() const
    {
        return mGraphType;
    }

    friend std::ostream& operator<<(std::ostream& os, const Graph<T>& graph)
    {
        for(auto it = graph.adjacencyMap.begin(); it != graph.adjacencyMap.end(); ++it) {
            os << it->first << ": ";
            for(auto& v : it->second)
                os << *v.first << " ";
            os << std::endl;
        }
        return os;
    }

private:
    std::unordered_map<T, std::vector<std::pair<T*, float>>> adjacencyMap;
    GraphType mGraphType;
    std::size_t mEdgesCount;
};
}
#endif // GRAPH_HPP
