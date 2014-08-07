//
//  dijkstras.h
//  Path planning
//
//  Created by Megamind on 8/1/14.
//  Copyright (c) 2014 ARSENL. All rights reserved.
//

#ifndef Path_planning_dijkstras_h
#define Path_planning_dijkstras_h

#include <unordered_map>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>

using namespace std;

class Graph
{
    unordered_map<int, const unordered_map<int, float>> vertices;
    
public:
    void add_vertex(int name, const unordered_map<int, float>& edges)
    {
        vertices.insert(unordered_map<int, const unordered_map<int, float>>::value_type(name, edges));
    }
    
    vector<int> shortest_path(int start, int finish)
    {
        unordered_map<int, float> distances;
        unordered_map<int, int> previous;
        vector<int> nodes;
        vector<int> path;
        
        auto comparator = [&] (int left, int right) { return distances[left] > distances[right]; };

        for (auto& vertex : vertices)
        {
            if (vertex.first == start)
            {
                distances[vertex.first] = 0;
            }
            else
            {
                distances[vertex.first] = numeric_limits<float>::max();
            }
            
            nodes.push_back(vertex.first);
            push_heap(begin(nodes), end(nodes), comparator);
        }
        while (!nodes.empty())
        {
            pop_heap(begin(nodes), end(nodes), comparator);
            int smallest = nodes.back();
            nodes.pop_back();
            
            if (smallest == finish)
            {
                while (previous.find(smallest) != end(previous))
                {
                    path.push_back(smallest);
                    smallest = previous[smallest];
                }
                
                break;
            }
            
            if (distances[smallest] == numeric_limits<int>::max())
            {
                break;
            }
            
            for (auto& neighbor : vertices[smallest])
            {
                int alt = distances[smallest] + neighbor.second;
                if (alt < distances[neighbor.first])
                {
                    distances[neighbor.first] = alt;
                    previous[neighbor.first] = smallest;
                    make_heap(begin(nodes), end(nodes), comparator);
                }
            }
        }
        
        return path;
    }
};

#endif
