/****************************************************************************************
* File: queue.h
*
* Description: Header file for maze.cpp
*
* Created: 2/20/2014, by Richard Habeeb
****************************************************************************************/

#ifndef QUEUE_INCLUDED_H
#define QUEUE_INCLUDED_H

/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/
#define QUEUE_LENGTH 256 

/*---------------------------------------------------------------------------------------
*                                        TYPES
*--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
*                                       CLASSES
*--------------------------------------------------------------------------------------*/

/******************************************************************************
* Class: Queue
*
* Description:	A non-exapanding implementation of a queue structure.
*				(this should except on overflows)
******************************************************************************/
template <class T>
class Queue
{
public: // methods

	Queue(void);
	~Queue(void);
	
	uint32_t get_count(void) const
	{
		return count;
	}

	// add and element to the queue
	Queue* Enqueue
	(
		T* item
	);

	// remove an element from the queue
	T* Dequeue(void);

	// return the top element of the queue
	T* Peek(void);

	// reset the queue pointers
	Queue* Reset(void);

public: // fields

public: // methods

private: // fields

	T* elements[QUEUE_LENGTH];
	uint32_t start;
	uint32_t end;
	uint32_t count;

};

/*---------------------------------------------------------------------------------------
*                                    CLASS METHODS
*--------------------------------------------------------------------------------------*/

/*****************************************************************************
* Function: Queue - Constructor
*
* Description: Initializes fields for new instance of queue object.
*****************************************************************************/
template <class T> inline
Queue<T>::Queue()
{
	Reset();
} // Queue()

/*****************************************************************************
* Function: Queue - Destructor
*
* Description: 
*****************************************************************************/
template <class T> inline
Queue<T>::~Queue()
{
} // ~Queue()

/*****************************************************************************
* Function: Enqueue
*
* Description: add and element to the queue
*****************************************************************************/
template <class T> inline
Queue<T>* Queue<T>::Enqueue(T* item)
{
	elements[(start++)%QUEUE_LENGTH] = item;
	count++;
	return this;
} // Enqueue()

/*****************************************************************************
* Function: Dequeue
*
* Description: removed from the queue
*****************************************************************************/
template <class T> inline
T* Queue<T>::Dequeue(void)
{
	if (count == 0)
	{
		return _NULL;
	}
	else
	{
		count--;
		return elements[(end++)%QUEUE_LENGTH];
	}
} // Dequeue()

/*****************************************************************************
* Function: Peek
*
* Description: return the front element of the queue
*****************************************************************************/
template <class T> inline
T* Queue<T>::Peek(void)
{
	if (count == 0)
	{
		return _NULL;
	}
	else
	{
		return elements[(end)%QUEUE_LENGTH];
	}
} // Peek()

/*****************************************************************************
* Function: Reset
*
* Description: reset all the indexes and the counter
*****************************************************************************/
template <class T> inline
Queue<T>* Queue<T>::Reset(void)
{
	start = 0;
	end = 0;
	count = 0;
	return this;
} // Reset()

#endif //QUEUE_INCLUDED_H