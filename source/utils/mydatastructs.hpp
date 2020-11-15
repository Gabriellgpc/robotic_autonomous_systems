#pragma once
#include <map>
#include <set>
#include <list>
#include <tuple>

/********************************************************/
/*********************** Graph **************************/
/********************************************************/
template <class T>
struct MyGraphEdge
{
    T src, dest;
    double cost;
};

template <class T>
class MyGraph
{
public:
    MyGraph();
    MyGraph(const std::list<MyGraphEdge<T>> &edges);

    void add_edge(const MyGraphEdge<T> &edge);
    bool exist(const MyGraphEdge<T> &edge);
    bool exist(const T &node);

    static void printMyGraph(const MyGraph &graph);

private:
    std::map<T, std::set<std::tuple<T, double>>> adjlist;
};

/********************************************************/
/********************* Tree *****************************/
/********************************************************/
template <class T>
class MyTree
{
public:
    MyTree();
    MyTree(const T &data);
    ~MyTree();

    void add_child(MyTree<T> &new_node);
    void set_father(MyTree<T> &new_father);
public:    
    T my_data;
    MyTree<T> *my_father;
    std::list<MyTree> my_children;
};