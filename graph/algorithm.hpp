#include "Graph.hpp"
#include <queue>
#include <stack>
#include <unordered_map>

namespace grf
{

template <typename T> std::vector<T*> bfs(const Graph<T>& g, const T& v)
{
    grf::Graph<T>* graphPtr = const_cast<grf::Graph<T>*>(&g);
    std::vector<T*> visited;

    if(graphPtr->exists(v)) {

        visited.reserve(graphPtr->size());
        T* vertexPtr = graphPtr->getVertex(v);

        std::queue<T*> queue;
        std::unordered_map<T*, std::size_t> visitedMap;

        visited.emplace_back(vertexPtr);
        visitedMap.insert(std::make_pair(vertexPtr, 0));

        for(auto e : graphPtr->getEdges(v))
            queue.push(e);

        while(!queue.empty()) {

            T* vertexPtr = queue.front();
            queue.pop();

            if(visitedMap.find(vertexPtr) == visitedMap.end()) {
                visitedMap.insert(std::make_pair(vertexPtr, 0));
                visited.emplace_back(vertexPtr);

                for(auto e : graphPtr->getEdges(*vertexPtr))
                    if(visitedMap.find(e) == visitedMap.end())
                        queue.push(e);
            }
        }
    }
    return visited;
}

template <typename T> std::vector<T*> dfs(const grf::Graph<T>& g, const T& v)
{
    grf::Graph<T>* graphPtr = const_cast<grf::Graph<T>*>(&g);
    std::vector<T*> visited;

    if(graphPtr->exists(v)) {
        visited.reserve(graphPtr->size());
        std::stack<T*> stack;
        std::unordered_map<T*, std::size_t> visitedMap;
        T* vertexPtr = graphPtr->getVertex(v);

        visitedMap.insert(std::make_pair(vertexPtr, 0));
        visited.emplace_back(vertexPtr);

        for(auto it = graphPtr->getEdges(*vertexPtr).rbegin(); it != graphPtr->getEdges(*vertexPtr).rend(); ++it)
            stack.push(*it);

        while(!stack.empty()) {
            T* vertexPtr = stack.top();
            stack.pop();

            if(visitedMap.find(vertexPtr) == visitedMap.end()) {
                visitedMap.insert(std::make_pair(vertexPtr, 0));
                visited.emplace_back(vertexPtr);

                for(auto it = graphPtr->getEdges(*vertexPtr).rbegin(); it != graphPtr->getEdges(*vertexPtr).rend();
                    ++it) {
                    if(visitedMap.find(*it) == visitedMap.end())
                        stack.push(*it);
                }
            }
        }
    }
    return visited;
}

template <typename T> std::vector<T*> bfsPath(const grf::Graph<T>& g, const T& vertexA, const T& vertexB)
{
    grf::Graph<T>* graphPtr = const_cast<grf::Graph<T>*>(&g);
    std::vector<T*> path;

    if(graphPtr->exists(vertexA) && graphPtr->exists(vertexB)) {
        std::unordered_map<T*, T*> mapPath;
        std::unordered_map<T*, std::size_t> visitedMap;
        std::queue<T*> queue;
        auto itVertex = graphPtr->getVertex(vertexA);
        auto itVertexB = graphPtr->getVertex(vertexB);

        visitedMap.insert(std::make_pair(itVertex, 0));
        std::vector<std::pair<T*, float>> edges = graphPtr->getEdgesWithWeight(*itVertex);

        for(auto it = edges.rbegin(); it != edges.rend(); ++it) {
            queue.push(it->first);
        }

        bool forceExit = false;
        while(!queue.empty()) {

            if(forceExit)
                break;

            T* stackVertex = queue.front();
            queue.pop();

            if(graphPtr->exists(*stackVertex)) {
                visitedMap.insert(std::make_pair(stackVertex, 0));

                if(stackVertex == itVertexB)
                    break;

                std::vector<std::pair<T*, float>> edges = graphPtr->getEdgesWithWeight(*stackVertex);

                for(auto it = edges.rbegin(); it != edges.rend(); ++it) {
                    if(visitedMap.find(it->first) == visitedMap.end()) {
                        queue.push(it->first);

                        mapPath[it->first] = stackVertex;
                        if(it->first == itVertexB) {
                            forceExit = true;
                            break;
                        }
                    }
                }
            }
        }

        T* curr = itVertexB;
        while(curr != itVertex) {
            path.emplace_back(curr);
            if(mapPath.find(curr) == mapPath.end())
                break;
            else
                curr = mapPath.find(curr)->second;
        }

        if(path.size() == 1) {
            std::vector<std::pair<T*, float>> edges = graphPtr->getEdgesWithWeight(*itVertex);

            auto it = std::find_if(edges.begin(), edges.end(),
                [itVertexB](const std::pair<T*, float>& vertex) { return vertex.first == itVertexB; });
            if(it == edges.end())
                path.clear();
        }
        if(!path.empty()) {
            path.emplace_back(itVertex);
            std::reverse(path.begin(), path.end());
        }
    }

    return path;
}

template <typename T> std::vector<T*> dijkstra(const Graph<T>& g, const T& vertexA, const T& vertexB)
{
    grf::Graph<T>* graphPtr = const_cast<grf::Graph<T>*>(&g);
    std::vector<T*> path;
    T* vertexBPtr = graphPtr->getVertex(vertexB);
    T* vertexAPtr = graphPtr->getVertex(vertexA);

    if(vertexAPtr != nullptr && vertexBPtr != nullptr) {
        typedef std::pair<float, T*> PQItem;
        std::unordered_map<T*, std::pair<T*, float>> mapPath;
        std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;

        pq.push(std::make_pair(0.f, vertexAPtr));
        mapPath.insert(std::make_pair(vertexAPtr, std::make_pair(vertexAPtr, 0.f)));

        while(!pq.empty()) {
            T* vertex = pq.top().second;
            pq.pop();

            if(*vertex == *vertexBPtr) {
                break;
            }

            std::vector<std::pair<T*, float>> vertices = graphPtr->getEdgesWithWeight(*graphPtr->getVertex(*vertex));

            for(std::size_t i = 0; i < vertices.size(); ++i) {
                T* ptr = graphPtr->getVertex(*vertices[i].first);
                float weight = mapPath[vertex].second + vertices[i].second;
                if(mapPath.find(ptr) == mapPath.end()) {
                    mapPath.insert(std::make_pair(ptr, std::make_pair(vertex, weight)));
                    pq.push(std::make_pair(weight, ptr));
                } else if(mapPath[ptr].second > weight) {
                    mapPath[ptr].second = weight;
                    pq.push(std::make_pair(weight, ptr));
                }
            }
        }
        path.reserve(mapPath.size());
        path.emplace_back(vertexBPtr);
        T* vertex = mapPath.find(vertexBPtr)->second.first;

        while(vertex != vertexAPtr) {
            path.emplace_back(vertex);
            vertex = mapPath.find(vertex)->second.first;
        }

        path.emplace_back(vertexAPtr);
        std::reverse(path.begin(), path.end());
    }

    return path;
}
}
