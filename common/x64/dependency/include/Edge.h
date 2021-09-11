/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Edge.h
 * @brief  The class definition for the "Edge" in Graph Theory.
 *******************************************************************************
 */
#ifndef YGEO_EDGE_H_
#define YGEO_EDGE_H_

#include "Vertex.h"

namespace YGEO {

class Vertex;

class Edge {


public:
	/**
	 * Constructor.
	 * @param from The from vertex of this edge.
	 * @param to The to vertex of this edge.
	 *
	 * Notes:
	 * After the construction, the edge instance will hold the from/to vertex pointer,
	 * and the from/to vertex will also hold the out/in edge pointer to this instance.
	 */
	Edge(Vertex *from, Vertex *to);

	/**
	 * Destructor.
	 *
	 * Notes:
	 * During the destruction, connections to both from/to vertices
	 * will be detached.
	 */
	virtual ~Edge();

	/**
	 * Get from-vertex of this edge.
	 * @return The from-vertex of this dege.
	 */
	virtual Vertex* getFrom() {return pFrom_;};

	/**
	 * Get to-vertex of this edge.
	 * @return The to-vertex of this dege.
	 */
	virtual Vertex* getTo() {return pTo_;};

#if 0
	/**
	 * Set the weight of this edge.
	 * @return void.
	 */
	virtual void setWeight(double weight)
	{
		weight_ = weight;
	};

	/**
	 * Get the weight this edge.
	 * @return The weight of this vertex.
	 */
	virtual double getWeight() const
	{
		return weight_;
	};
#endif
friend class Vertex;

protected:
	/**
	 * Detach the connection to the from/to vertex. After this method is invoded,
	 * the from/to vertex will no longer hold the out/in edge pointer to this instance..
	 */
	virtual void detach();

private:
	Edge();
	Edge(const Edge& obj);
	Edge& operator= (const Edge& obj);

	Vertex *pFrom_;
	Vertex *pTo_;

	// double weight_ = 0.0;
};

} /* namespace YGEO */

#endif /* YGEO_EDGE_H_ */
