/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2018
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Graph.h
 * @brief  The class definition for the "Graph" in Graph Theory.
 *
 * Rivision History:
 *      Date              Submitter         Description
 *      2018.03.19        Tony Xiong		Initialization
 *******************************************************************************
 */

#ifndef COM_YGOMI_ROADDB_GMAPI_GRAPH_H_
#define COM_YGOMI_ROADDB_GMAPI_GRAPH_H_

#include <list>
#include <vector>
#include "./Vertex.h"
#include "./Edge.h"

namespace YGEO {

class Graph {
public:
	/**
	 * Default constructor.
	 */
	Graph();

	/**
	 * Destructor.
	 *
	 * Notes:
	 * During the destruction, all added vertices and edges will be deleted.
	 */
	virtual ~Graph();

	/**
	 * Get all vertices in this graph.
	 *
	 * @return A list containing all vertices.
	 */
	std::list<Vertex*>& getVertices() {return vertices_;};

	/**
	 * Get all edges in this graph.
	 *
	 * @return A list containing all edges.
	 */
	std::list<Edge*>& getEdges() {return edges_;};

	/**
	 * Add a vertex into the graph.
	 *
	 * @param vertex the vertex to be added.
	 * @return True if added, or false if the vertex already exists.
	 */
	bool addVertex(Vertex* vertex);

	/**
	 * Add a edge into the graph.
	 *
	 * @param edge the edge to be added.
	 * @return True if added, or false if the edge already exists.
	 */
	bool addEdge(Edge* edge);

	/**
	 * Clear all vertices and edges (all will be deleted).
	 */
	void clear();
#if 0
	/**
	 * get the shortest path from source to destination.
	 * 
	 * @param source: the source vertex
	 * @param destination: the destination vertex
	 * @param path: the shortest path
	 * @param pathLen: the length of shortest path
	 * @return true if there exist a shortest path, false if not
	 */
	bool Dijstra(Vertex *source, Vertex *destination, std::vector<const Vertex *> &shortestPath, double &pathLen);

	/**
	 * get the k-th shortest path from source to destination.
	 * 
	 * @param source: the source vertex
	 * @param destination: the destination vertex
	 * @param shortestPath: a shortest path
	 * @param pathLen: the length of a shortest path
	 * @return true if there exist next shortest path, false if not
	 */
	bool KSPOneStep(Vertex *source, Vertex *destination, std::vector<const Vertex *> &shortestPath, double &pathLen);

	/**
	 * initilization for KSP algorithm
	 */
	void KSPInit();
#endif
private:
	std::list<Vertex*> vertices_;
	std::list<Edge*> edges_;

	// std::list<double> pathsBackupLen;
	// std::vector<std::vector<const Vertex *>> pathsChosen;
	// std::list<std::vector<const Vertex *>> pathsBackup;
};

} /* namespace YGEO */

#endif /* COM_YGOMI_ROADDB_GMAPI_GRAPH_H_ */
