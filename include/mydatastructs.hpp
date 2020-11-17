#pragma once
#include <list>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <stack>          // std::stack


template<class T>
class MyTreeNode;
template<class T>
class MyGraph;

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
    static std::stack<T> a_start(MyGraph<T> graph, const T &src, const T &dest, double heuristic(int,void*));
private:
    std::unordered_map<T, std::unordered_set<std::tuple<T, double>>> adjlist;
};



/********************************************************/
/********************* Tree *****************************/
/********************************************************/
template <class T>
class MyTreeNode
{
public:
    MyTreeNode();
    MyTreeNode(const T &data);
    ~MyTreeNode();

    void add_child(MyTreeNode<T> &new_node);
    void set_father(MyTreeNode<T> &new_father);
public:    
    T my_data;
    MyTreeNode<T> *my_father;
    std::list<MyTreeNode<T>> my_children;
};