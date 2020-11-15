#include "mydatastructs.hpp"

#include <algorithm>
#include <list>
#include <iostream>

template <class T>
MyGraph<T>::MyGraph()
{}

template <class T>
MyGraph<T>::MyGraph(const std::list<MyGraphEdge<T>> &edges)
{
    for (auto edge_it = edges.begin(); edge_it != edges.end(); edge_it++)
    {
        adjlist[edge_it->src].insert(std::make_tuple(edge_it->dest, edge_it->cost));
        adjlist[edge_it->dest].insert(std::make_tuple(edge_it->src, edge_it->cost));
    }
}

template <class T>
void MyGraph<T>::add_edge(const MyGraphEdge<T> &edge)
{
    adjlist[edge.src].insert(std::make_tuple(edge.dest, edge.cost));
    adjlist[edge.dest].insert(std::make_tuple(edge.src, edge.cost));
}

template<class T>
bool MyGraph<T>::exist(const MyGraphEdge<T> &edge)
{
    return adjlist[edge.src].count(edge.dest) > 0;
}

template<class T>
bool MyGraph<T>::exist(const T &node)
{
    return adjlist.count(node) > 0;
}

template <class T>
void MyGraph<T>::printMyGraph(const MyGraph<T> &graph)
{
    for(auto m_it = graph.adjlist.begin(); m_it != graph.adjlist.end(); m_it++)
    {
        for(auto s_it = m_it->second.begin(); s_it != m_it->second.end(); s_it++)
        {
            std::cout << '{' << m_it->first << ',' << std::get<0>(*s_it) << ',' << std::get<1>(*s_it) << "}\n";
        }
    }
}

/********************************************************/
/********************* Tree *****************************/
/********************************************************/
template<class T>
MyTree<T>::MyTree()
{
    my_father = NULL;
    my_data = T();
}

template<class T>
MyTree<T>::MyTree(const T &data)
{
    my_data = data;
    my_father = NULL;
}

template<class T>
MyTree<T>::~MyTree()
{

}

template<class T>
void MyTree<T>::add_child(MyTree<T> &new_node)
{
    new_node.my_father = this;
    my_children.push_back(new_node);
}

template<class T>
void MyTree<T>::set_father(MyTree<T> &new_father)
{
    my_father = &new_father;
    new_father.add_child(*this);
}