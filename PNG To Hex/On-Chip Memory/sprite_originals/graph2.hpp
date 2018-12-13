/**
* @return The number of vertices in the Graph
*/
template <class V, class E>
unsigned int Graph<V,E>::size() const {
  // TODO: Part 2
  return vertexMap.size();
}


/**
* @return Returns the degree of a given vertex.
* @param v Given vertex to return degree.
*/
template <class V, class E>
unsigned int Graph<V,E>::degree(const V & v) const {
  // TODO: Part 2
  std::string val = "";
  for(auto it : vertexMap){
    if(it->second == v){
      val = it->first;
      break;
    }
  }
  return (adjList.find(val)->second).size();
}


/**
* Inserts a Vertex into the Graph by adding it to the Vertex map and adjacency list
* @param key The key of the Vertex to insert
* @return The inserted Vertex
*/
template <class V, class E>
V & Graph<V,E>::insertVertex(std::string key) {
  // TODO: Part 2
  V & v = *(new V(key));
  vertexMap.insert({key, v});
  std::list<edgeListIter> l;
  adjList.insert({key, l});
  return v;
}


/**
* Removes a given Vertex
* @param v The Vertex to remove
*/
template <class V, class E>
void Graph<V,E>::removeVertex(const std::string & key) {
  // TODO: Part 2
  vertexMap.erase(key);
  adjList.erase(key);
}


/**
* Inserts an Edge into the adjacency list
* @param v1 The source Vertex
* @param v2 The destination Vertex
* @return The inserted Edge
*/
template <class V, class E>
E & Graph<V,E>::insertEdge(const V & v1, const V & v2) {
  // TODO: Part 2
  E & e = *(new E(v1, v2));
  edgeList.push_front(e);
  edgeListIter it = edgeList.begin();
  adjList[v1.key()].push_front(it);
  adjList[v2.key()].push_front(it);
  return e;
}


/**
* Removes an Edge from the Graph
* @param key1 The key of the source Vertex
* @param key2 The key of the destination Vertex
*/
template <class V, class E>
void Graph<V,E>::removeEdge(const std::string key1, const std::string key2) {
  // TODO: Part 2
  for(auto e : edgeList)
    if(e.source().key() == key1 && e.dest().key() == key2)
      edgeList.remove(e);
  for(edgeListIter & it : adjList[key1])
    if(((Edge&)*it).source().key() == key1 && ((Edge&)*it).dest().key() == key2)
      removeEdge(it);
}


/**
* Removes an Edge from the adjacency list at the location of the given iterator
* @param it An iterator at the location of the Edge that
* you would like to remove
*/
template <class V, class E>
void Graph<V,E>::removeEdge(const edgeListIter & it) {
  // TODO: Part 2
  Edge e = *it;
  Vertex& v1 = (*it).source();
  Vertex& v2 = (*it).dest();
  adjList[v1].remove(it);
  adjList[v2].remove(it);
  edgeList.erase(it);
}


/**
* @param key The key of an arbitrary Vertex "v"
* @return The list edges (by reference) that are adjacent to "v"
*/
template <class V, class E>
const std::list<std::reference_wrapper<E>> Graph<V,E>::incidentEdges(const std::string key) const {
  // TODO: Part 2
  std::list<std::reference_wrapper<E>> edges;
  for(std::reference_wrapper<E> e : edgeList)
    if(e.get().source().key() == key || e.get().dest().key() == key)
      edges.push_back(e);
  return edges;
}


/**
* Return whether the two vertices are adjacent to one another
* @param key1 The key of the source Vertex
* @param key2 The key of the destination Vertex
* @return True if v1 is adjacent to v2, False otherwise
*/
template <class V, class E>
bool Graph<V,E>::isAdjacent(const std::string key1, const std::string key2) const {
  // TODO: Part 2
  std::list<edgeListIter> adj = adjList.at(key1);
  std::list<edgeListIter> adj2 = adjList.at(key2);
  std::list<edgeListIter> it;
  if(adj.size() > adj2.size())
    it = adj2;
  else
    it = adj;
  for(edgeListIter itr : it)
    if(((Edge&)*itr).source().key() == key1 && ((Edge&)*itr).dest().key() == key2)
      return true;
  return false;
}
