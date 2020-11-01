// #pragma once
// #include <vector>

// void polygon_to_vectorsXY(const Polygon2D &polygon, std::vector<double> &vertices_x, std::vector<double> &vertices_y)
// {
//     auto vertices = polygon.get_vertices();

//     for (auto it = vertices.begin(); it != vertices.end(); it++)
//     {
//         vertices_x.push_back(it->x());
//         vertices_y.push_back(it->y());
//     }

//     if (vertices.size() > 1.0)
//     {
//         vertices_x.push_back(vertices.front().x());
//         vertices_y.push_back(vertices.front().y());
//     }
// }