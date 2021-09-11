/*******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   vertex.h
 * @brief  The class definition for the "Vertex" in Graph Theory.
 *******************************************************************************
 */
#ifndef YGEO_VERTEX_H_
#define YGEO_VERTEX_H_

#include <list>
#include "Edge.h"

namespace YGEO {

class Edge;

class Vertex {


public:
	/**
	 * Default constructor.
	 */
	Vertex();

	/**
	 * Destructor.
	 *
	 * Notes:
	 * During the destruction, connections to all in/out edges
	 * will be detached.
	 */
	virtual ~Vertex();

	/**
	 * Get in-degree of this vertex.
	 * @return The in-degree of this vertex.
	 */
	virtual int getInDegree()
	{
		return inEdges_.size();
	}

	/**
	 * Get out-degree of this vertex.
	 * @return The out-degree of this vertex.
	 */
	virtual int getOutDegree()
	{
		return outEdges_.size();
	}

	/**
	 * Get in-edges of this vertex.
	 * @return The list of in-edges of this vertex.
	 */
	virtual std::list<Edge*>& getInEdges()
	{
		return inEdges_;
	};

	/**
	 * Get out-edges of this vertex.
	 * @return The list of out-edges of this vertex.
	 */
	virtual std::list<Edge*>& getOutEdges()
	{
		return outEdges_;
	};
#if 0
	/**
	 * Set the weight of this vertex.
	 * @return void.
	 */
	virtual void setWeight(double weight)
	{
		weight_ = weight;
	};

	/**
	 * Get the weight this vertex.
	 * @return The weight of this vertex.
	 */
	virtual double getWeight() const
	{
		return weight_;
	};
#endif
friend class Edge;

protected:

private:
	Vertex(const Vertex& obj);
	Vertex& operator= (const Vertex& obj);

	std::list<Edge*> inEdges_;
	std::list<Edge*> outEdges_;
	// double weight_ = 0.0;
};

} /* namespace YGEO */

#endif /* YGEO_VERTEX_H_ */
