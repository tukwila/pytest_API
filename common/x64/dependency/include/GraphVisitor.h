/*
 * GraphVisitor.h
 *
 *  Created on: Aug 20, 2017
 *      Author: Tony Xiong
 */
/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   GraphVisitor.h
 * @brief  This class implements a "Visitor" design pattern for graph traversal.
 *******************************************************************************
 */
#ifndef YGEO_GRAPHVISITOR_H_
#define YGEO_GRAPHVISITOR_H_

#include <list>
#include <set>
#include <map>
#include <queue>
#include <functional>

namespace YGEO {

template<class GraphType, class VertexType, class EdgeType>
class GraphVisitor {
public:
	/**
	 * The type definition for Vertex Action. Vertex Action is a callback function which is invoked
	 * when a vertex is being visited.
	 *
	 * @param visitor The visitor to which this action is attached to.
	 * @param vertex The vertex that the visitor is current at.
	 * @return Return ERRCODE_SUCCESS to continue traversal, while return ERRCODE_ABORT
	 * to abort the traveral.
	 */
	typedef std::function<int (const GraphVisitor &visitor, const VertexType &vertex)> VertexActionType;

	/**
	 * The type definition for Edge Action. Edge Action is a callback function which is invoked
	 * when an edge is being visited.
	 *
	 * @param visitor The visitor to which this action is attached to.
	 * @param edge The edge that the visitor is current at.
	 * @return Return ERRCODE_SUCCESS to continue traversal, while return ERRCODE_ABORT
	 * to abort the traveral.
	 */
	typedef std::function<int (const GraphVisitor &visitor, const EdgeType &edge)> EdgeActionType;

	/**
	 * The type definition for Vertex Guard. Vertex Guard is a callback function to check if a vertex
	 * is allowed to be visited or not. If the guard is satisified (true returned), the vertex is allowed
	 * to be visited, otherwise the vertex will be skipped.
	 *
	 * @param visitor The visitor to which this action is attached to.
	 * @param vertex The vertex that the visitor is going to visit.
	 * @return Return true to make the guard satisfied, false otherwise.
	 *
	 * Note if the vertex is skipped, the traversal path from this vertex will be pruned.
	 */
	typedef std::function<bool (const GraphVisitor &visitor, const VertexType &vertex)> VertexGuardType;

	/**
	 * The type definition for Edge Guard. Edge Guard is a callback function to check if an edge
	 * is allowed to be visited or not. If the guard is satisified (true returned), the edge is allowed
	 * to be visited, otherwise the edge will be skipped.
	 *
	 * @param visitor The visitor to which this action is attached to.
	 * @param edge The edge that the visitor is going to visit.
	 * @return Return true to make the guard satisfied, false otherwise.
	 *
	 * Note if the edge is skipped, the traversal path from the to-vertex of this edge will be pruned
	 * (including the to-vertex itself).
	 */
	typedef std::function<bool (const GraphVisitor &visitor, const EdgeType &edge)> EdgeGuardType;

	/**
	 * The case of success
	 */
	static const int ERRCODE_SUCCESS = 0;

	/**
	 * The case of abortion. If this error code is returned from vertex/edge action, the traversal will
	 * be aborted. I.E.: No more vertex/edge will be visited, and the traveral/search will be returned
	 * with ERRCODE_ABORT.
	 */
	static const int ERRCODE_ABORT = -1;

	/**
	 * The case when the requested operation is not supported.
	 */
	static const int ERRCODE_NOT_SUPPORTED = -2;

//	static const int ERRCODE_GUARD_FAILURE = 1001; //Failed by the guard function
//	static const int ERRCODE_VISITED = 1002; //The vertex/edge is alreay visited

	/**
	 * Constructor
	 *
	 * @param graph the pointer to the graph
	 */
	GraphVisitor(GraphType *graph = nullptr) :
		_graph(graph),
		_startVertex(nullptr),
		_currentVertex(nullptr),
		_vertexAction(nullptr),
		_edgeAction(nullptr),
		_vertexGuard(nullptr),
		_edgeGuard(nullptr)
	{
	};

	/**
	 * Destructor
	 */
	virtual ~GraphVisitor() {};

	/**
	 * Get graph to which this visitor is attached.
	 *
	 * @return The pointer to the graph to which this visitor is attached.
	 */
	virtual const GraphType* getGraph() const {return _graph;};

	/**
	 * Get the vertex from which this visitor is going to start traversing.
	 *
	 * @return The pointer to the vertex from which this visitor is going to start traversing.
	 *
	 * Notes:
	 * Initially the start-vertex will be null.
	 * The start-vertex will be set by depthFirstSearch()/breathFirstSearch().
	 */
	virtual const VertexType* getStartVertex() const {return _startVertex;};

	/**
	 * Get the vertex at which this visitor currently sits during a traversal.
	 *
	 * @return The pointer to the vertex at which this visitor currently sits during a traversal.
	 *
	 * Notes:
	 * Initially the current-vertex will be null.
	 * Null will be returned if not in process of traversal.
	 */
	virtual const VertexType* getCurrentVertex() const {return _currentVertex;};

	/**
	 * Get visited vertices of a traversal.
	 *
	 * @return A set containing all visited vertices.
	 *
	 * Notes:
	 * If no traversal is ever invoked, an empty set will be returned.
	 * Otherwise, a set containing the visited vertices in the latest traversal
	 * will be returned.
	 */
	virtual const std::set<const VertexType*>& getVisitedVertices() const {return _visitedVertices;};

	/**
	 * Get visited edges of a traversal.
	 *
	 * @return A set containing all visited edges.
	 *
	 * Notes:
	 * If no traversal is ever invoked, an empty set will be returned.
	 * Otherwise, a set containing the visited edges in the latest traversal
	 * will be returned.
	 */
	virtual const std::set<const EdgeType*>& getVisitedEdges() const {return _visitedEdges;};

	/**
	 * Get the DFS stack in a Depth First Search. The DFS stack is a sequence of edges
	 * starting from the start vertex to the current vertex during a DFS traversal.
	 *
	 * @return A reference to a list containing the DFS stack.
	 *
	 * Notes:
	 * This method is for DFS only. In case of BFS, the returned list will contain no element.
	 * This method is supposed to be invoked in the vertex/edge action.
	 * When traversal is finished, the list will be cleared empty.
	 * The stack is pushed/popped from the front of the list.
	 * The returned list is maintained by the GraphVisitor.
	 */
	virtual const std::list<const EdgeType*>& getDFSStack() const {return _dfsStack;};

//	//Get Breath Fist Search Stack (push at front, pop at back).
//	const std::deque<const VertexType*>& getDFSQueue() const {return _wfsQueue;};

//	const VertexActionType getVertexAction() const {return _vertexAction;};
//	const EdgeActionType getEdgeAction() const {return _edgeAction;};
//	const VertexGuardType getVertexVisitGuard() const {return _vertexGuard;};
//	const EdgeGuardType getEdgeVisitGuard() const {return _edgeGuard;};

	/**
	 * Set the callback vertex action.
	 *
	 * @param action the vertex action
	 */
	void setVertexAction(VertexActionType &action) {_vertexAction = action;};

	/**
	 * Set the callback edge action.
	 *
	 * @param action the edge action
	 */
	void setEdgeAction(EdgeActionType &action)	{_edgeAction = action;};

	/**
	 * Set the vertex guard.
	 *
	 * @param guard the vertex guard
	 */
	void setVertexGuard(VertexGuardType &guard) {_vertexGuard = guard;};

	/**
	 * Set the edge guard.
	 *
	 * @param guard the edge guard
	 */
	void setEdgeGuard(EdgeGuardType &guard) {_edgeGuard = guard;};

	/**
	 * Start a Depth First Search/Traversal.
	 *
	 * @param startVertex the vertex where the Depth First Search/Traversal starts from.
	 * @param isEdgeTraversal true if traversal on edge, or false if traversal on vertex.
	 *        When traversal on edge, no edge shall be visited more than once;
	 *        When traversal on vertex, no vertex shall be visited more than once.
	 *
	 * @return ERRCODE_SUCCESS if successfully finished, or ERRCODE_ABORT if aborted due to
	 * exception/callback action indication.
	 */
	virtual int depthFirstSearch(VertexType* startVertex, bool isEdgeTraversal = true)
	{
		int errorCode = ERRCODE_SUCCESS;

		_visitedVertices.clear();
		_visitedEdges.clear();
		_dfsStack.clear();

		_startVertex = startVertex;
		if (_startVertex == nullptr) {
			return ERRCODE_SUCCESS;
		}

		if (_vertexGuard != nullptr && !_vertexGuard(*this, *_startVertex)) {
//			return ERRCODE_GUARD_FAILURE;
			return ERRCODE_SUCCESS;
		}

		errorCode = doDepthFirstSearch(_startVertex, isEdgeTraversal);

		return errorCode;
	};

	/**
	 * Start a Breadth First Search/Traversal.
	 *
	 * @param startVertex the vertex where the Breadth First Search/Traversal starts from.
	 * @param isEdgeTraversal true if traversal on edge, or false if traversal on vertex.
	 *        When traversal on edge, no edge shall be visited more than once;
	 *        When traversal on vertex, no vertex shall be visited more than once.
	 *
	 * @return ERRCODE_SUCCESS if successfully finished, or ERRCODE_ABORT if aborted due to
	 * exception/callback action indication.
	 */
	virtual int breadthFirstSearch(VertexType* startVertex, bool isEdgeTraversal = true)
	{
		int errorCode = ERRCODE_SUCCESS;

		_visitedVertices.clear();
		_visitedEdges.clear();

		_startVertex = startVertex;
		if (_startVertex == nullptr) {
			return ERRCODE_SUCCESS;
		}

		if (_vertexGuard != nullptr && !_vertexGuard(*this, *_startVertex)) {
//			return ERRCODE_GUARD_FAILURE;
			return ERRCODE_SUCCESS;
		}

		errorCode = doBreadthFirstSearch(_startVertex, isEdgeTraversal);

		return errorCode;
	};


protected:
	GraphType *_graph;
	VertexType *_startVertex;
	VertexType *_currentVertex;

	VertexActionType _vertexAction;
	EdgeActionType _edgeAction;
	VertexGuardType _vertexGuard;
	EdgeGuardType _edgeGuard;

	std::set<const VertexType*> _visitedVertices;
	std::set<const EdgeType*> _visitedEdges;
	std::list<const EdgeType*> _dfsStack; //depth first traverse stack
//	std::deque<const VertexType*> _wfsQueue; //width first traverse queue

private:
	GraphVisitor() = delete;
	GraphVisitor(const GraphVisitor& obj) = delete;
	GraphVisitor& operator= (const GraphVisitor& obj) = delete;

	int doDepthFirstSearch(VertexType *startVertex, bool isEdgeTraversal)
	{
		int errorCode = ERRCODE_SUCCESS;
		VertexType *nextVertex = nullptr;
		EdgeType* edge = nullptr;
		typename std::list<EdgeType*>::reverse_iterator edgeIter;

		////////////////////////////////////////////////////////////////
		//A helper list used for traversal algorithm, which keeps track of the vertices.
		//It will be taken as a stack for DF Traversal, or a queue for BF Traversal.
		////////////////////////////////////////////////////////////////
		std::list<VertexType*> vertexList;

		////////////////////////////////////////////////////////////////
		//A helper list used for traversal algorithm, which keeps track of the edges.
		//It will be taken as a stack for DF Traversal, or a queue for BF Traversal.
		////////////////////////////////////////////////////////////////
		std::list<EdgeType*> edgeList;

		//Push _startVertex into vertexList to start the algorithm.
		vertexList.push_front(startVertex);

		//Push a pseudo edge into edgeList so as to make algorithm efficient
		edgeList.push_front(nullptr);

		while (!vertexList.empty()) {
			//The front of vertexList is the current vertex to be visited.
			_currentVertex = vertexList.front();
			//The front of edgeList is the edge via which the current vertex is traversed.
			edge = edgeList.front();

			if (isEdgeTraversal) {
				if (edge == nullptr) {
					if (_visitedVertices.find(_currentVertex) == _visitedVertices.end()) {
						//This must be the first visit of the root vertex, do nothing.
					} else {
						vertexList.pop_front();
						edgeList.pop_front();
						if (!_dfsStack.empty()) {
							_dfsStack.pop_back();
						}
						continue;
					}
				} else {
					if (_visitedEdges.find(edge) != _visitedEdges.end()) {
						//Do back-stack for the edge has been visited.
						vertexList.pop_front();
						edgeList.pop_front();
						if (!_dfsStack.empty()) {
							_dfsStack.pop_back();
						}
						continue;
					}
				}
			} else {
				if (_visitedVertices.find(_currentVertex) != _visitedVertices.end()) {
					//Do back-stack for the vertex has been visited.
					vertexList.pop_front();
					edgeList.pop_front();
					if (!_dfsStack.empty()) {
						_dfsStack.pop_back();
					}
					continue;
				}
			}

			if (edge != nullptr) {
				//edge is currently being visited. Put it into _visitedEdges.
				_visitedEdges.insert(edge);
				//The edge to _currentVertex is currently being visited. Put it into _dfsStack.
				_dfsStack.push_back(edge);
				//Invoke edge action callback if any.
				if (_edgeAction != nullptr && _edgeAction(*this, *edge) == ERRCODE_ABORT) {
					_currentVertex = nullptr;
					return ERRCODE_ABORT;
				}
			} else {
				//This must be the pseudo edge added
			}

			//_currentVertex is currently being visited. Put it into _visitedVertices.
			_visitedVertices.insert(_currentVertex);
			//Invoke vertex action callback if any.
			if (_vertexAction != nullptr) {
				if (_vertexAction(*this, *_currentVertex) == ERRCODE_ABORT) {
					_currentVertex = nullptr;
					return ERRCODE_ABORT;
				}
			}

			////////////////////////////////////////////////////////////////
			//Push successor vertices into _dfsHelperStack if any.
			////////////////////////////////////////////////////////////////
			for (edgeIter = _currentVertex->getOutEdges().rbegin();
					edgeIter != _currentVertex->getOutEdges().rend();
					edgeIter++) {
				edge = *edgeIter;
				nextVertex = edge->getTo();
				if (nextVertex == nullptr) { //invalid edge
					//Shall not reach here!
					continue;
				}

				if (isEdgeTraversal) {
					//If edge is visited, skip it.
					if (_visitedEdges.find(edge) != _visitedEdges.end()) {
						continue;
					}
				} else {
					//If nextVertex is visited, skip it.
					if (_visitedVertices.find(nextVertex) != _visitedVertices.end()) {
						continue;
					}
				}

				//If the edge to nextGuard has guard and the guard check fails, skip nextVertex.
				if (_edgeGuard != nullptr && !_edgeGuard(*this, *edge)) {
					continue;
				}

				//If nextVertex has guard and the guard check fails, skip it.
				if (_vertexGuard != nullptr && !_vertexGuard(*this, *nextVertex)) {
					continue;
				}

				//keep track of the edge via which the nextVertex is traversed.
				edgeList.push_front(edge);

				//Keep track of the nextVertex.
				vertexList.push_front(nextVertex);
			}
		}

		_currentVertex = nullptr;

		return errorCode;
	};


	int doBreadthFirstSearch(VertexType *startVertex, bool isEdgeTraversal)
	{
		int errorCode = ERRCODE_SUCCESS;
		VertexType *nextVertex = nullptr;
		EdgeType* edge = nullptr;
		typename std::list<EdgeType*>::iterator edgeIter;

		////////////////////////////////////////////////////////////////
		//A helper list used for traversal algorithm, which keeps track of the vertices.
		//It will be taken as a stack for DF Traversal, or a queue for BF Traversal.
		////////////////////////////////////////////////////////////////
		std::list<VertexType*> vertexList;

		////////////////////////////////////////////////////////////////
		//A helper list used for traversal algorithm, which keeps track of the edges.
		//It will be taken as a stack for DF Traversal, or a queue for BF Traversal.
		////////////////////////////////////////////////////////////////
		std::list<EdgeType*> edgeList;

		//Push _startVertex into vertexList to start the algorithm.
		vertexList.push_back(startVertex);

		//Push a pseudo edge into edgeList so as to make algorithm efficient
		edgeList.push_back(nullptr);

		while (!vertexList.empty()) {
			//The front of vertexList is the current vertex to be visited.
			_currentVertex = vertexList.front();
			vertexList.pop_front();

			//The front of edgeList is the edge via which the current vertex is traversed.
			edge = edgeList.front();
			edgeList.pop_front();

			if (edge != nullptr) {
                if(_visitedEdges.find(edge) !=_visitedEdges.end())
                { //If the edge has been visited, skip it.
                    continue;
                }

                if (_edgeGuard != nullptr && !_edgeGuard(*this, *edge))
                { //If the edge guard fails, skip it.
                    continue;
                }

                //edge is currently being visited. Put it into _visitedEdges.
				_visitedEdges.insert(edge);

				//Invoke edge action callback if any.
				if (_edgeAction != nullptr && _edgeAction(*this, *edge) == ERRCODE_ABORT) {
					_currentVertex = nullptr;
					return ERRCODE_ABORT;
				}
			} else {
				//This must be the pseudo edge added
			}

            if (_currentVertex == nullptr)
            { //Invalid vertex. Shall not reach here.
                continue;
            }

            if(!isEdgeTraversal &&
               _visitedVertices.find(_currentVertex) !=_visitedVertices.end())
            { //If it is traversal on vertex, and _currentVertex has been visited, skip it.
                continue;
            }

            if (_vertexGuard != nullptr && !_vertexGuard(*this, *_currentVertex))
            { //If the vertex guard fails, skip it.
                continue;
            }

            //_currentVertex is currently being visited. Put it into _visitedVertices.
            _visitedVertices.insert(_currentVertex);

            //Invoke vertex action callback if any.
            if (_vertexAction != nullptr) {
                if (_vertexAction(*this, *_currentVertex) == ERRCODE_ABORT) {
                    _currentVertex = nullptr;
                    return ERRCODE_ABORT;
                }
            }

			////////////////////////////////////////////////////////////////
			//Process successor vertices if any.
			////////////////////////////////////////////////////////////////
			for (auto edge : _currentVertex->getOutEdges()) {
					nextVertex = edge->getTo();
					if (nextVertex == nullptr) { //invalid edge
						//Shall not reach here!
						continue;
					}

                    if (isEdgeTraversal) {
                        //If edge is visited, skip it.
                        if (_visitedEdges.find(edge) != _visitedEdges.end()) {
                            continue;
                        }
                    } else {
                        //If nextVertex is visited, skip it.
                        if (_visitedVertices.find(nextVertex) != _visitedVertices.end()) {
                            continue;
                        }
                    }

					//If the edge to nextGuard has guard and the guard check fails, skip nextVertex.
					if (_edgeGuard != nullptr && !_edgeGuard(*this, *edge)) {
						continue;
					}

					//If nextVertex has guard and the guard check fails, skip it.
					if (_vertexGuard != nullptr && !_vertexGuard(*this, *nextVertex)) {
						continue;
					}

					//keep track of the edge via which the nextVertex is traversed.
					edgeList.push_back(edge);

					//Keep track of the nextVertex.
					vertexList.push_back(nextVertex);
			}
		}

		_currentVertex = nullptr;

		return errorCode;
	};

};

} /* namespace YGEO */

#endif /* YGEO_GRAPHVISITOR_H_ */
