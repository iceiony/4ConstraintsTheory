/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#include "GraphicsManager.h"
#include "ListenerBase.h"

ListenerBase::ListenerBase(GraphicsManager* const scene, const char* const listenerName)
{
	NewtonWorld* const world = scene->GetNewton();
	void* const prelistener = NewtonWorldAddPreListener (world, listenerName, this, PreUpdate, NULL);
	NewtonWorldAddPostListener (world, listenerName, this, PostUpdate, Destroy);

	NewtonWorldListenerSetBodyDestroyCallback (world, prelistener, OnBodyDetroy);
}

ListenerBase::~ListenerBase()
{
}

void ListenerBase::OnBodyDestroy (NewtonBody* const body)
{
}

void ListenerBase::PreUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep)
{
	ListenerBase* const me = (ListenerBase*) listenerUserData;
	me->PreUpdate(world, timestep);
}


void ListenerBase::PostUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep)
{
	ListenerBase* const me = (ListenerBase*) listenerUserData;
	me->PostUpdate(world, timestep);
}

void ListenerBase::Destroy (const NewtonWorld* const world, void* const listenerUserData)
{
	ListenerBase* const me = (ListenerBase*) listenerUserData;
	delete me;
}

void ListenerBase::OnBodyDetroy (const NewtonWorld* const world, void* const listener, NewtonBody* const body)
{
	ListenerBase* const me = (ListenerBase*) NewtonWorldGetListenerUserData(world, listener);
	me->OnBodyDestroy(body);
}
