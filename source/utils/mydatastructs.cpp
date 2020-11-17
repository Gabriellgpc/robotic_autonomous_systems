#include "mydatastructs.hpp"

#include <iostream>   //std::ostream
#include <queue>      // std::priority_queue
#include <functional> // std::greater
#include <stack>      // std::stack

template <class T>
MyGraph<T>::MyGraph()
{
}

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

template <class T>
bool MyGraph<T>::exist(const MyGraphEdge<T> &edge)
{
    return adjlist[edge.src].count(edge.dest) > 0;
}

template <class T>
bool MyGraph<T>::exist(const T &node)
{
    return adjlist.count(node) > 0;
}

template <class T>
void MyGraph<T>::printMyGraph(const MyGraph<T> &graph)
{
    for (auto m_it = graph.adjlist.begin(); m_it != graph.adjlist.end(); m_it++)
    {
        for (auto s_it = m_it->second.begin(); s_it != m_it->second.end(); s_it++)
        {
            std::cout << '{' << m_it->first << ',' << std::get<0>(*s_it) << ',' << std::get<1>(*s_it) << "}\n";
        }
    }
}

template <class T>
std::stack<T> MyGraph<T>::a_start(MyGraph<T> graph, const T &src, const T &dest, double heuristic(int, void *))
{
    std::priority_queue<MyTreeNode<std::tuple<T, double>>,
                        std::vector<MyTreeNode<std::tuple<T, double>>>,
                        std::greater<MyTreeNode<std::tuple<T, double>>>>
        min_heap;

    MyTreeNode<std::tuple<T, double>> curr_node, *new_node;
    std::unordered_set<MyTreeNode<std::tuple<T, double>>> marked;
    std::unordered_set<T> visited;
    MyTreeNode<std::tuple<T, double>> root({src, 0.0});

    T curr_T;
    double curr_cost;

    //add T inicial com custo zero
    min_heap.push(root);
    marked.insert(root);

    while (!min_heap.empty())
    {
        //pega o nó de menor custo
        curr_node = min_heap.top();
        curr_T = std::get<0>(curr_node.my_data);
        curr_cost = std::get<1>(curr_node.my_data);

        visited.insert(curr_T);
        min_heap.pop();
        if (marked.count(curr_node) <= 0)
            continue;
        //remove dos marcados
        marked.erase(curr_node);
        //verifica se chegou no destino
        if (curr_T == dest)
            break;

        //percorrer os nós adjacentes de curr_node
        static auto adj_nodes = graph.adjlist[curr_T].begin();
        for (auto node_it = adj_nodes.begin(); node_it != nodes.end(); node_it++)
        {
            if (visited.count(std::get<0>(*node_it)) > 0)
                continue;
            //se nó não visitado (não esta na min_heap e in marked) => adicione-o
            if (marked.count(*node_it) > 0)
            {
                //adicionar o nó na arvore apontando para curr_node
                *new_node = new MyTreeNode<std::tuple<T, double>>();
                new_node->my_data = *node_it;
                std::get<1> new_node->my_data += curr_cost;

                curr_node->add_child(*new_node);
                //adiciona na min_heap
                min_heap.push(new_node);
                //marca como visitado
                marked.insert(new_node);
            }
            else //caso ja visitado
            {
                auto node_marked = marked.find(*node_it);
                //g(N')
                double g_node_marked = std::get<1>(node_marked->my_data);
                //g(N) + K(N,N')
                double g_curr_to_adj = curr_cost + graph.adjlist[curr_T][std::get<0>(*node_it)];
                //verifica se o caminho pelo nó analisado ficou mais custoso
                if (g_node_marked > g_curr_to_adj)
                {
                    marked.erase(*node_it);

                    //atualiza o custo e troca os ponteiros para fazer N' passar por N
                    std::get<1>(node_marked->my_data) = g_curr_to_adj;
                    curr_node->add_child(*node_marked);

                    marked.insert(*node_marked);
                    min_heap.push(*node_marked);
                }
            }
        }
    }

    std::stack<T> min_path;

    if (curr_T == dest)
    {
        std::cout << "Busca de caminho finalizada com sucesso! custo total:" << std::get<1>(curr_node.my_data) << '\n';
        while (curr_T != src)
        {
            min_path.push(curr_T);
            //next
            curr_node = *curr_node.my_father;
            curr_T = std::get<0>(curr_node.my_data);
        }
    }
    else
    {
        std::cout << "Falha ao encontrar o caminho!\n";
    }

    return min_path;
}

/********************************************************/
/********************* Tree *****************************/
/********************************************************/
template <class T>
MyTreeNode<T>::MyTreeNode()
{
    my_father = NULL;
    my_data = T();
}

template <class T>
MyTreeNode<T>::MyTreeNode(const T &data)
{
    my_data = data;
    my_father = NULL;
}

template <class T>
MyTreeNode<T>::~MyTreeNode()
{
}

template <class T>
void MyTreeNode<T>::add_child(MyTreeNode<T> &new_node)
{
    new_node.my_father = this;
    my_children.push_back(new_node);
}

template <class T>
void MyTreeNode<T>::set_father(MyTreeNode<T> &new_father)
{
    my_father = &new_father;
    new_father.add_child(*this);
}